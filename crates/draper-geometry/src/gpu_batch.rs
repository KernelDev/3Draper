// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! GPU-ready batch evaluation API for NURBS surfaces (ROADMAP_VISION_2036 §7).
//!
//! Per Directive 5 (GPU-First Thinking): this module provides a
//! Structure-of-Arrays (SOA) data layout for NURBS surfaces that is
//! designed for direct porting to WebGPU Compute Shaders (WGSL).
//!
//! Key design principles:
//! - **SOA layout**: control points stored as flat f32 arrays (x[], y[], z[])
//!   instead of Vec<Vec<Point3d>> — enables coalesced GPU memory access
//! - **No pointers**: all data is stored in contiguous Vec<f32> buffers
//! - **Batch evaluation**: evaluate N points on M surfaces in a single call
//! - **f32 precision**: GPU shaders typically use f32; this module matches
//!
//! Future: the same SOA layout can be directly uploaded to GPU storage
//! buffers and evaluated in a WGSL compute shader with minimal adaptation.

use crate::NurbsSurface;
use crate::Point3d;

/// SOA (Structure of Arrays) representation of a NURBS surface.
///
/// This is the GPU-ready format: all data is stored as flat f32 arrays
/// that can be directly uploaded to a GPU storage buffer.
///
/// Layout:
/// - `cp_x`, `cp_y`, `cp_z`: control point coordinates, flattened as
///   `cp[v * n_u + u]` where `n_u` is the number of control points in U
/// - `weights`: same layout as control points
/// - `u_knots`, `v_knots`: knot vectors
/// - `u_degree`, `v_degree`: polynomial degrees
/// - `n_u`, `n_v`: control point grid dimensions
#[derive(Clone, Debug)]
pub struct NurbsSurfaceSOA {
    /// Control point X coordinates: `cp_x[v * n_u + u]`
    pub cp_x: Vec<f32>,
    /// Control point Y coordinates: `cp_y[v * n_u + u]`
    pub cp_y: Vec<f32>,
    /// Control point Z coordinates: `cp_z[v * n_u + u]`
    pub cp_z: Vec<f32>,
    /// Weights: `weights[v * n_u + u]`
    pub weights: Vec<f32>,
    /// U-direction knot vector
    pub u_knots: Vec<f32>,
    /// V-direction knot vector
    pub v_knots: Vec<f32>,
    /// Degree in U direction
    pub u_degree: u32,
    /// Degree in V direction
    pub v_degree: u32,
    /// Number of control points in U direction
    pub n_u: u32,
    /// Number of control points in V direction
    pub n_v: u32,
}

impl NurbsSurfaceSOA {
    /// Convert from the standard NurbsSurface (AOS) to SOA layout.
    ///
    /// The standard `NurbsSurface` uses `control_points[u][v]` (nested Vec),
    /// which is AOS (Array of Structures). This method flattens it into
    /// separate x/y/z/weight arrays for GPU compatibility.
    pub fn from_nurbs(surface: &NurbsSurface) -> Self {
        let n_u = surface.control_points.len();
        let n_v = surface.control_points.first().map(|r| r.len()).unwrap_or(0);
        let total = n_u * n_v;

        let mut cp_x = Vec::with_capacity(total);
        let mut cp_y = Vec::with_capacity(total);
        let mut cp_z = Vec::with_capacity(total);
        let mut weights = Vec::with_capacity(total);

        for v in 0..n_v {
            for u in 0..n_u {
                let cp = &surface.control_points[u][v];
                let w = surface.weights.get(u).and_then(|r| r.get(v)).copied().unwrap_or(1.0);
                cp_x.push(cp.x as f32);
                cp_y.push(cp.y as f32);
                cp_z.push(cp.z as f32);
                weights.push(w as f32);
            }
        }

        Self {
            cp_x,
            cp_y,
            cp_z,
            weights,
            u_knots: surface.u_knots.iter().map(|&k| k as f32).collect(),
            v_knots: surface.v_knots.iter().map(|&k| k as f32).collect(),
            u_degree: surface.u_degree as u32,
            v_degree: surface.v_degree as u32,
            n_u: n_u as u32,
            n_v: n_v as u32,
        }
    }

    /// Evaluate the surface at a single (u, v) parameter.
    ///
    /// Uses the De Boor algorithm in f32 arithmetic (matching GPU precision).
    /// Returns (x, y, z) as a tuple for easy GPU shader porting.
    pub fn evaluate(&self, u: f32, v: f32) -> (f32, f32, f32) {
        let p = self.u_degree as usize;
        let q = self.v_degree as usize;
        let n_u = self.n_u as usize;
        let n_v = self.n_v as usize;

        // Clamp to knot range
        let u_min = self.u_knots.get(p).copied().unwrap_or(0.0);
        let u_max = self.u_knots.get(self.u_knots.len().saturating_sub(p + 1)).copied().unwrap_or(1.0);
        let v_min = self.v_knots.get(q).copied().unwrap_or(0.0);
        let v_max = self.v_knots.get(self.v_knots.len().saturating_sub(q + 1)).copied().unwrap_or(1.0);
        let u_c = u.clamp(u_min, u_max);
        let v_c = v.clamp(v_min, v_max);

        // Find knot spans
        let k_u = find_knot_span_soa(&self.u_knots, p, u_c, n_u);
        let k_v = find_knot_span_soa(&self.v_knots, q, v_c, n_v);

        // Step 1: Evaluate B-spline in V for each U-row → p+1 intermediate points
        let mut inter_x = vec![0.0f32; p + 1];
        let mut inter_y = vec![0.0f32; p + 1];
        let mut inter_z = vec![0.0f32; p + 1];
        let mut inter_w = vec![0.0f32; p + 1];

        for i in 0..=p {
            let row = k_u.saturating_sub(p) + i;
            if row >= n_u {
                continue;
            }
            // Collect q+1 control points in V for this row
            let mut vx = vec![0.0f32; q + 1];
            let mut vy = vec![0.0f32; q + 1];
            let mut vz = vec![0.0f32; q + 1];
            let mut vw = vec![0.0f32; q + 1];
            for j in 0..=q {
                let col = k_v.saturating_sub(q) + j;
                let col = col.min(n_v.saturating_sub(1));
                let idx = col * n_u + row;
                vx[j] = self.cp_x[idx] * self.weights[idx];
                vy[j] = self.cp_y[idx] * self.weights[idx];
                vz[j] = self.cp_z[idx] * self.weights[idx];
                vw[j] = self.weights[idx];
            }
            // De Boor in V
            de_boor_soa(&mut vx, &mut vy, &mut vz, &mut vw, &self.v_knots, q, k_v, v_c);
            inter_x[i] = vx[q];
            inter_y[i] = vy[q];
            inter_z[i] = vz[q];
            inter_w[i] = vw[q];
        }

        // Step 2: De Boor in U on intermediate points
        de_boor_soa(&mut inter_x, &mut inter_y, &mut inter_z, &mut inter_w, &self.u_knots, p, k_u, u_c);

        let w = inter_w[p];
        if w.abs() < 1e-15 {
            return (0.0, 0.0, 0.0);
        }
        let x = inter_x[p] / w;
        let y = inter_y[p] / w;
        let z = inter_z[p] / w;

        // NaN/Inf guard
        if x.is_finite() && y.is_finite() && z.is_finite() {
            (x, y, z)
        } else {
            (0.0, 0.0, 0.0)
        }
    }

    /// Batch evaluate the surface at multiple (u, v) parameter pairs.
    ///
    /// This is the GPU-ready API: takes flat arrays of U and V parameters
    /// and produces flat arrays of X, Y, Z results. The same function
    /// signature can be directly ported to a WGSL compute shader:
    ///
    /// ```ignore
    /// // WGSL equivalent:
    /// @compute @workgroup_size(64)
    /// fn evaluate_batch(@builtin(global_invocation_id) gid: vec3<u32>) {
    ///     let idx = gid.x;
    ///     if idx >= params.count { return; }
    ///     let u = u_params[idx];
    ///     let v = v_params[idx];
    ///     // ... De Boor evaluation ...
    ///     out_x[idx] = x;
    ///     out_y[idx] = y;
    ///     out_z[idx] = z;
    /// }
    /// ```
    pub fn evaluate_batch(
        &self,
        u_params: &[f32],
        v_params: &[f32],
        out_x: &mut [f32],
        out_y: &mut [f32],
        out_z: &mut [f32],
    ) {
        let n = u_params.len().min(v_params.len()).min(out_x.len()).min(out_y.len()).min(out_z.len());
        for i in 0..n {
            let (x, y, z) = self.evaluate(u_params[i], v_params[i]);
            out_x[i] = x;
            out_y[i] = y;
            out_z[i] = z;
        }
    }

    /// Generate a regular UV grid for batch evaluation.
    ///
    /// Produces `n_u × n_v` parameter pairs covering the surface's
    /// parametric domain. Useful for generating Steiner grids via
    /// batch evaluation.
    pub fn generate_uv_grid(&self, n_u: usize, n_v: usize) -> (Vec<f32>, Vec<f32>) {
        let p = self.u_degree as usize;
        let q = self.v_degree as usize;
        let u_min = self.u_knots.get(p).copied().unwrap_or(0.0);
        let u_max = self.u_knots.get(self.u_knots.len().saturating_sub(p + 1)).copied().unwrap_or(1.0);
        let v_min = self.v_knots.get(q).copied().unwrap_or(0.0);
        let v_max = self.v_knots.get(self.v_knots.len().saturating_sub(q + 1)).copied().unwrap_or(1.0);

        let mut u_params = Vec::with_capacity(n_u * n_v);
        let mut v_params = Vec::with_capacity(n_u * n_v);

        for vi in 0..n_v {
            let v = if n_v > 1 {
                v_min + (v_max - v_min) * vi as f32 / (n_v - 1) as f32
            } else {
                (v_min + v_max) * 0.5
            };
            for ui in 0..n_u {
                let u = if n_u > 1 {
                    u_min + (u_max - u_min) * ui as f32 / (n_u - 1) as f32
                } else {
                    (u_min + u_max) * 0.5
                };
                u_params.push(u);
                v_params.push(v);
            }
        }

        (u_params, v_params)
    }

    /// Total byte size of all data buffers (for GPU buffer allocation).
    pub fn total_bytes(&self) -> usize {
        let cp_bytes = self.cp_x.len() * 4; // f32 = 4 bytes
        let weight_bytes = self.weights.len() * 4;
        let knot_bytes = (self.u_knots.len() + self.v_knots.len()) * 4;
        let metadata_bytes = 6 * 4; // 6 u32 fields
        cp_bytes * 3 + weight_bytes + knot_bytes + metadata_bytes
    }
}

/// Find the knot span index k such that knots[k] <= t < knots[k+1].
///
/// This is the SOA/f32 version of `find_knot_span` in surface.rs.
/// Designed for direct WGSL porting.
fn find_knot_span_soa(knots: &[f32], degree: usize, t: f32, n: usize) -> usize {
    if n == 0 {
        return 0;
    }
    if t >= knots[n] {
        return n - 1;
    }
    if t <= knots[degree] {
        return degree;
    }
    let mut low = degree;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

/// De Boor algorithm step (SOA/f32 version).
///
/// Performs in-place De Boor subdivision on the given arrays.
/// Matches the algorithm in surface.rs::de_boor_step() (Piegl & Tiller
/// "The NURBS Book" Algorithm A2.2).
fn de_boor_soa(
    x: &mut [f32],
    y: &mut [f32],
    z: &mut [f32],
    w: &mut [f32],
    knots: &[f32],
    degree: usize,
    span: usize,
    t: f32,
) {
    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let i = span - degree + j;
            let alpha = if i + degree + 1 - r < knots.len() && i < knots.len() {
                let denom = knots[i + degree + 1 - r] - knots[i];
                if denom.abs() < 1e-15 {
                    0.0
                } else {
                    (t - knots[i]) / denom
                }
            } else {
                0.0
            };
            let beta = 1.0 - alpha;
            x[j] = alpha * x[j] + beta * x[j - 1];
            y[j] = alpha * y[j] + beta * y[j - 1];
            z[j] = alpha * z[j] + beta * z[j - 1];
            w[j] = alpha * w[j] + beta * w[j - 1];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_soa_from_nurbs() {
        let nurbs = NurbsSurface {
            u_degree: 2,
            v_degree: 2,
            control_points: vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0), Point3d::new(0.0, 2.0, 0.0)],
                vec![Point3d::new(1.0, 0.0, 1.0), Point3d::new(1.0, 1.0, 2.0), Point3d::new(1.0, 2.0, 1.0)],
                vec![Point3d::new(2.0, 0.0, 0.0), Point3d::new(2.0, 1.0, 0.0), Point3d::new(2.0, 2.0, 0.0)],
            ],
            weights: vec![vec![1.0; 3]; 3],
            u_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        };
        let soa = NurbsSurfaceSOA::from_nurbs(&nurbs);
        assert_eq!(soa.n_u, 3);
        assert_eq!(soa.n_v, 3);
        assert_eq!(soa.cp_x.len(), 9);
        assert_eq!(soa.u_degree, 2);
    }

    #[test]
    fn test_soa_evaluate_center() {
        let nurbs = NurbsSurface {
            u_degree: 1,
            v_degree: 1,
            control_points: vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0)],
                vec![Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 0.0)],
            ],
            weights: vec![vec![1.0; 2]; 2],
            u_knots: vec![0.0, 0.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        };
        let soa = NurbsSurfaceSOA::from_nurbs(&nurbs);
        // At u=0.5, v=0.5, a bilinear surface evaluates to the centroid
        let (x, y, z) = soa.evaluate(0.5, 0.5);
        assert!((x - 0.5).abs() < 1e-5, "x = {}, expected 0.5", x);
        assert!((y - 0.5).abs() < 1e-5, "y = {}, expected 0.5", y);
        assert!(z.abs() < 1e-5, "z = {}, expected 0.0", z);
    }

    #[test]
    fn test_soa_batch_evaluate() {
        let nurbs = NurbsSurface {
            u_degree: 1,
            v_degree: 1,
            control_points: vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 1.0, 0.0)],
                vec![Point3d::new(1.0, 0.0, 0.0), Point3d::new(1.0, 1.0, 0.0)],
            ],
            weights: vec![vec![1.0; 2]; 2],
            u_knots: vec![0.0, 0.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        };
        let soa = NurbsSurfaceSOA::from_nurbs(&nurbs);

        let (u_params, v_params) = soa.generate_uv_grid(3, 3);
        let mut out_x = vec![0.0f32; 9];
        let mut out_y = vec![0.0f32; 9];
        let mut out_z = vec![0.0f32; 9];

        soa.evaluate_batch(&u_params, &v_params, &mut out_x, &mut out_y, &mut out_z);

        // Corner (0,0) should be (0,0,0)
        assert!(out_x[0].abs() < 1e-5);
        assert!(out_y[0].abs() < 1e-5);
        // Corner (1,1) should be (1,1,0)
        assert!((out_x[8] - 1.0).abs() < 1e-5);
        assert!((out_y[8] - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_soa_nan_guard() {
        let nurbs = NurbsSurface {
            u_degree: 1,
            v_degree: 1,
            control_points: vec![
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0)],
                vec![Point3d::new(0.0, 0.0, 0.0), Point3d::new(0.0, 0.0, 0.0)],
            ],
            weights: vec![vec![0.0; 2]; 2], // Zero weights → division by zero
            u_knots: vec![0.0, 0.0, 1.0, 1.0],
            v_knots: vec![0.0, 0.0, 1.0, 1.0],
            u_closed: false,
            v_closed: false,
        };
        let soa = NurbsSurfaceSOA::from_nurbs(&nurbs);
        let (x, y, z) = soa.evaluate(0.5, 0.5);
        // Should return (0,0,0) — not NaN/Inf
        assert!(x.is_finite());
        assert!(y.is_finite());
        assert!(z.is_finite());
    }
}
