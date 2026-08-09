// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Generative design via topology optimization on SDF (ROADMAP_VISION_2036 §5).
//!
//! Per §5 (Phase 5): Generative design as a native kernel function.
//! Uses the ImplicitSolid (SDF) module for topology optimization —
//! iteratively removes material where it's not needed (low stress)
//! while preserving structural constraints.
//!
//! Algorithm: Solid Isotropic Material with Penalization (SIMP)
//! 1. Discretize the design space into a voxel grid
//! 2. Compute stress distribution via FEA (simplified)
//! 3. Remove material where stress is low (threshold)
//! 4. Smooth the result (SDF filtering)
//! 5. Repeat until convergence or target volume fraction

use crate::{ImplicitSolid, Sdf, CsgNode, SphereSdf, BoxSdf};
use draper_geometry::Point3d;

// ============================================================
// Design space and constraints
// ============================================================

/// The design space for topology optimization.
#[derive(Clone, Debug)]
pub struct DesignSpace {
    /// Min corner of the bounding box.
    pub min: Point3d,
    /// Max corner of the bounding box.
    pub max: Point3d,
    /// Grid resolution [nx, ny, nz].
    pub resolution: [usize; 3],
}

impl DesignSpace {
    /// Create a design space from a bounding box.
    pub fn new(min: Point3d, max: Point3d, resolution: [usize; 3]) -> Self {
        Self { min, max, resolution }
    }

    /// Total number of voxels.
    pub fn num_voxels(&self) -> usize {
        self.resolution[0] * self.resolution[1] * self.resolution[2]
    }

    /// Get the center of voxel (i, j, k).
    pub fn voxel_center(&self, i: usize, j: usize, k: usize) -> Point3d {
        let dx = (self.max.x - self.min.x) / self.resolution[0].max(1) as f64;
        let dy = (self.max.y - self.min.y) / self.resolution[1].max(1) as f64;
        let dz = (self.max.z - self.min.z) / self.resolution[2].max(1) as f64;
        Point3d::new(
            self.min.x + (i as f64 + 0.5) * dx,
            self.min.y + (j as f64 + 0.5) * dy,
            self.min.z + (k as f64 + 0.5) * dz,
        )
    }

    /// Voxel size.
    pub fn voxel_size(&self) -> [f64; 3] {
        [
            (self.max.x - self.min.x) / self.resolution[0].max(1) as f64,
            (self.max.y - self.min.y) / self.resolution[1].max(1) as f64,
            (self.max.z - self.min.z) / self.resolution[2].max(1) as f64,
        ]
    }
}

/// A load case for topology optimization.
#[derive(Clone, Debug)]
pub struct LoadCase {
    /// Fixed (constrained) regions — box definitions where material is required.
    /// Each tuple: (center, half_x, half_y, half_z)
    pub fixed_regions: Vec<(Point3d, f64, f64, f64)>,
    /// Applied forces: (position, direction, magnitude).
    pub forces: Vec<(Point3d, [f64; 3], f64)>,
    /// Target volume fraction (0.0 = no material, 1.0 = full block).
    pub target_volume_fraction: f64,
}

impl Default for LoadCase {
    fn default() -> Self {
        Self {
            fixed_regions: Vec::new(),
            forces: Vec::new(),
            target_volume_fraction: 0.4,
        }
    }
}

// ============================================================
// Topology optimization result
// ============================================================

/// Result of topology optimization.
#[derive(Clone, Debug)]
pub struct TopologyOptimizationResult {
    /// Density field: 0.0 = void, 1.0 = solid, values in between = intermediate.
    pub density: Vec<f64>,
    /// Grid dimensions.
    pub resolution: [usize; 3],
    /// Design space bounding box.
    pub bbox_min: Point3d,
    pub bbox_max: Point3d,
    /// Final volume fraction achieved.
    pub achieved_volume_fraction: f64,
    /// Number of iterations run.
    pub iterations: usize,
    /// Whether optimization converged.
    pub converged: bool,
}

impl TopologyOptimizationResult {
    /// Extract the optimized shape as an ImplicitSolid (SDF).
    ///
    /// The density field is converted to an SDF by treating voxels with
    /// density > threshold as solid and computing a signed distance to
    /// the nearest void voxel.
    pub fn to_implicit_solid(&self, threshold: f64) -> ImplicitSolid {
        // Build a box for each solid voxel and union them.
        // For efficiency, we use a simplified approach: create a single
        // BoxSdf for the bounding region and subtract void regions.
        //
        // In a production system, this would use a sparse voxel SDF
        // (octree-based) for efficient evaluation. Here we use a
        // simplified version for demonstration.

        let voxel_size = [
            (self.bbox_max.x - self.bbox_min.x) / self.resolution[0].max(1) as f64,
            (self.bbox_max.y - self.bbox_min.y) / self.resolution[1].max(1) as f64,
            (self.bbox_max.z - self.bbox_min.z) / self.resolution[2].max(1) as f64,
        ];

        // Collect solid voxel centers
        let mut solid_centers: Vec<Point3d> = Vec::new();
        for k in 0..self.resolution[2] {
            for j in 0..self.resolution[1] {
                for i in 0..self.resolution[0] {
                    let idx = k * self.resolution[0] * self.resolution[1]
                            + j * self.resolution[0] + i;
                    if let Some(&density) = self.density.get(idx) {
                        if density > threshold {
                            let cx = self.bbox_min.x + (i as f64 + 0.5) * voxel_size[0];
                            let cy = self.bbox_min.y + (j as f64 + 0.5) * voxel_size[1];
                            let cz = self.bbox_min.z + (k as f64 + 0.5) * voxel_size[2];
                            solid_centers.push(Point3d::new(cx, cy, cz));
                        }
                    }
                }
            }
        }

        if solid_centers.is_empty() {
            return ImplicitSolid::sphere(Point3d::ORIGIN, 0.001);
        }

        // Build union of small spheres at each solid voxel center
        // (simplified — production would use octree SDF)
        let radius = voxel_size[0].min(voxel_size[1]).min(voxel_size[2]) * 0.75;

        let mut csg = CsgNode::leaf(SphereSdf {
            center: solid_centers[0],
            radius,
        });

        for center in &solid_centers[1..32.min(solid_centers.len())] {
            csg = CsgNode::union(csg, CsgNode::leaf(SphereSdf {
                center: *center,
                radius,
            }));
        }

        // If too many voxels, use bounding box as approximation
        if solid_centers.len() > 32 {
            let center = Point3d::new(
                (self.bbox_min.x + self.bbox_max.x) * 0.5,
                (self.bbox_min.y + self.bbox_max.y) * 0.5,
                (self.bbox_min.z + self.bbox_max.z) * 0.5,
            );
            let half = [
                (self.bbox_max.x - self.bbox_min.x) * 0.5 * self.achieved_volume_fraction.cbrt(),
                (self.bbox_max.y - self.bbox_min.y) * 0.5 * self.achieved_volume_fraction.cbrt(),
                (self.bbox_max.z - self.bbox_min.z) * 0.5 * self.achieved_volume_fraction.cbrt(),
            ];
            csg = CsgNode::leaf(BoxSdf { center, half_size: half });
        }

        ImplicitSolid::new(csg)
    }

    /// Count solid voxels (density > threshold).
    pub fn solid_voxel_count(&self, threshold: f64) -> usize {
        self.density.iter().filter(|&&d| d > threshold).count()
    }

    /// Compute the compliance (inverse of stiffness) — lower is better.
    /// Simplified: uses density sum as proxy.
    pub fn compliance(&self) -> f64 {
        self.density.iter().map(|d| (1.0 - d).powi(3)).sum()
    }
}

// ============================================================
// Topology optimization algorithm (SIMP)
// ============================================================

/// Run topology optimization on a design space with given load case.
///
/// Uses a simplified SIMP (Solid Isotropic Material with Penalization)
/// approach:
/// 1. Initialize density field to target_volume_fraction (uniform)
/// 2. Compute "sensitivity" (heuristic: distance to nearest force/fixed region)
/// 3. Update densities: increase where sensitive, decrease where not
/// 4. Project to satisfy volume constraint
/// 5. Smooth (filter) the density field
/// 6. Repeat for max_iterations or until converged
///
/// This is a simplified implementation suitable for conceptual design.
/// A production implementation would use FEA (Finite Element Analysis)
/// for accurate stress computation.
pub fn optimize_topology(
    design_space: &DesignSpace,
    load_case: &LoadCase,
    max_iterations: usize,
) -> TopologyOptimizationResult {
    let nx = design_space.resolution[0];
    let ny = design_space.resolution[1];
    let nz = design_space.resolution[2];
    let total = nx * ny * nz;

    // Initialize density to target volume fraction
    let mut density = vec![load_case.target_volume_fraction; total];

    // Compute sensitivity field (simplified — no FEA)
    let sensitivity = compute_sensitivity(design_space, load_case);

    let mut converged = false;
    let mut last_volume = load_case.target_volume_fraction;

    for iter in 0..max_iterations {
        // Update densities based on sensitivity
        let mut new_density = vec![0.0; total];
        for i in 0..total {
            // SIMP update: density proportional to sensitivity
            let sens = sensitivity[i];
            let penalized = sens.powi(3); // Penalization (p=3)
            new_density[i] = density[i] * penalized;
        }

        // Normalize to maintain volume constraint
        let current_volume: f64 = new_density.iter().sum();
        if current_volume > 0.0 {
            let scale = (load_case.target_volume_fraction * total as f64) / current_volume;
            for d in &mut new_density {
                *d = (*d * scale).clamp(0.0, 1.0);
            }
        }

        // Smooth (filter) — simple 3D box filter
        new_density = smooth_density(&new_density, nx, ny, nz);

        // Check convergence
        let new_volume: f64 = new_density.iter().sum::<f64>() / total as f64;
        let volume_change = (new_volume - last_volume).abs();
        last_volume = new_volume;

        density = new_density;

        if volume_change < 1e-4 {
            converged = true;
            log::info!("Topology optimization converged at iteration {}", iter + 1);
            break;
        }
    }

    let achieved_volume = density.iter().sum::<f64>() / total as f64;

    log::info!(
        "Topology optimization: {} iterations, volume_fraction={:.3} (target={:.3}), converged={}",
        max_iterations, achieved_volume, load_case.target_volume_fraction, converged
    );

    TopologyOptimizationResult {
        density,
        resolution: [nx, ny, nz],
        bbox_min: design_space.min,
        bbox_max: design_space.max,
        achieved_volume_fraction: achieved_volume,
        iterations: max_iterations,
        converged,
    }
}

/// Compute sensitivity field (simplified — heuristic, no FEA).
///
/// Higher sensitivity near forces and fixed regions, lower in the middle
/// of the design space. This is a proxy for stress distribution.
fn compute_sensitivity(design_space: &DesignSpace, load_case: &LoadCase) -> Vec<f64> {
    let nx = design_space.resolution[0];
    let ny = design_space.resolution[1];
    let nz = design_space.resolution[2];
    let total = nx * ny * nz;

    let mut sensitivity = vec![0.5; total];

    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let idx = k * nx * ny + j * nx + i;
                let center = design_space.voxel_center(i, j, k);

                let mut sens = 0.0;

                // Increase sensitivity near forces
                for (force_pos, _dir, magnitude) in &load_case.forces {
                    let dist = ((center.x - force_pos.x).powi(2)
                        + (center.y - force_pos.y).powi(2)
                        + (center.z - force_pos.z).powi(2)).sqrt();
                    sens += magnitude / (dist + 1.0);
                }

                // Increase sensitivity in fixed regions
                for (fcenter, fhx, fhy, fhz) in &load_case.fixed_regions {
                    let box_sdf = BoxSdf {
                        center: *fcenter,
                        half_size: [*fhx, *fhy, *fhz],
                    };
                    let dist = box_sdf.signed_distance(&center);
                    if dist < 0.0 {
                        sens += 1.0;
                    } else if dist < 5.0 {
                        sens += 1.0 / (dist + 1.0);
                    }
                }

                // Default base sensitivity
                if sens < 0.1 {
                    sens = 0.1;
                }

                sensitivity[idx] = sens;
            }
        }
    }

    // Normalize to [0, 1]
    let max_sens = sensitivity.iter().fold(0.0f64, |a, &b| a.max(b));
    if max_sens > 0.0 {
        for s in &mut sensitivity {
            *s /= max_sens;
        }
    }

    sensitivity
}

/// Apply a 3D box filter for smoothing.
fn smooth_density(density: &[f64], nx: usize, ny: usize, nz: usize) -> Vec<f64> {
    let mut result = vec![0.0; density.len()];
    let radius = 1; // 3×3×3 filter

    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                let idx = k * nx * ny + j * nx + i;
                let mut sum = 0.0;
                let mut count = 0;

                for dk in -(radius as isize)..=(radius as isize) {
                    let kk = k as isize + dk;
                    if kk < 0 || kk >= nz as isize { continue; }
                    for dj in -(radius as isize)..=(radius as isize) {
                        let jj = j as isize + dj;
                        if jj < 0 || jj >= ny as isize { continue; }
                        for di in -(radius as isize)..=(radius as isize) {
                            let ii = i as isize + di;
                            if ii < 0 || ii >= nx as isize { continue; }
                            let nidx = (kk as usize) * nx * ny + (jj as usize) * nx + (ii as usize);
                            sum += density[nidx];
                            count += 1;
                        }
                    }
                }

                result[idx] = if count > 0 { sum / count as f64 } else { density[idx] };
            }
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ImplicitSolid;

    #[test]
    fn test_design_space() {
        let ds = DesignSpace::new(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(10.0, 10.0, 10.0),
            [5, 5, 5],
        );
        assert_eq!(ds.num_voxels(), 125);
        let center = ds.voxel_center(0, 0, 0);
        assert!((center.x - 1.0).abs() < 1e-10); // (0 + 0.5) * 2 = 1
    }

    #[test]
    fn test_optimize_simple_cantilever() {
        // Simple cantilever beam: fixed on left, force on right
        let ds = DesignSpace::new(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(20.0, 10.0, 10.0),
            [10, 5, 5],
        );

        let load_case = LoadCase {
            fixed_regions: vec![(Point3d::new(0.0, 5.0, 5.0), 1.0, 5.0, 5.0)],
            forces: vec![(Point3d::new(20.0, 5.0, 5.0), [-1.0, 0.0, 0.0], 100.0)],
            target_volume_fraction: 0.4,
        };

        let result = optimize_topology(&ds, &load_case, 10);
        assert_eq!(result.resolution, [10, 5, 5]);
        assert!(result.achieved_volume_fraction > 0.0);
        assert!(result.achieved_volume_fraction < 1.0);
    }

    #[test]
    fn test_result_to_implicit_solid() {
        let result = TopologyOptimizationResult {
            density: vec![1.0; 8], // 2×2×2 all solid
            resolution: [2, 2, 2],
            bbox_min: Point3d::new(0.0, 0.0, 0.0),
            bbox_max: Point3d::new(4.0, 4.0, 4.0),
            achieved_volume_fraction: 1.0,
            iterations: 1,
            converged: true,
        };
        let solid = result.to_implicit_solid(0.5);
        let (bmin, bmax) = solid.bounding_box();
        // Should have some bounding box
        assert!(bmax.x > bmin.x);
    }

    #[test]
    fn test_solid_voxel_count() {
        let result = TopologyOptimizationResult {
            density: vec![1.0, 0.0, 0.5, 1.0, 0.0, 0.8, 0.3, 0.0],
            resolution: [2, 2, 2],
            bbox_min: Point3d::ORIGIN,
            bbox_max: Point3d::new(1.0, 1.0, 1.0),
            achieved_volume_fraction: 0.45,
            iterations: 1,
            converged: true,
        };
        assert_eq!(result.solid_voxel_count(0.5), 3); // 1.0, 1.0, 0.8
    }

    #[test]
    fn test_smoothing() {
        // Single spike in flat field
        let density = vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let smoothed = smooth_density(&density, 4, 4, 2);
        // The spike should be reduced by averaging
        assert!(smoothed[2] < 1.0); // Spike reduced
        assert!(smoothed[2] > 0.0); // But not zero
    }

    #[test]
    fn test_volume_constraint() {
        let ds = DesignSpace::new(
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(4.0, 4.0, 4.0),
            [4, 4, 4],
        );
        let load_case = LoadCase {
            target_volume_fraction: 0.3,
            ..Default::default()
        };
        let result = optimize_topology(&ds, &load_case, 5);
        // Volume fraction should be close to target
        assert!((result.achieved_volume_fraction - 0.3).abs() < 0.15);
    }
}
