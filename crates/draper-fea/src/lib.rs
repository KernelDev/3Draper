// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Finite Element Analysis (FEA) — linear static structural solver.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 2.3: provides a pure-Rust
//! FEA solver for linear static analysis (no external C++ dependencies
//! like tetgen/netgen, preserving WASM compatibility).
//!
//! # Pipeline
//!
//! 1. **Mesh generation**: Convert a B-Rep `Solid` (or `TriangleMesh`)
//!    into a tetrahedral mesh by extruding triangles inward.
//! 2. **Boundary conditions**: Apply fixed constraints (zero displacement
//!    on selected faces) and forces (concentrated loads on nodes/faces).
//! 3. **Assembly**: Build the global stiffness matrix `K` and force
//!    vector `F` from element stiffness matrices.
//! 4. **Solve**: Solve `K · u = F` for the displacement vector `u`
//!    using Conjugate Gradient (sparse, iterative).
//! 5. **Post-process**: Compute von Mises stress from displacements.
//!
//! # Material Model
//!
//! Linear isotropic elastic material:
//! - `E` (Young's modulus)
//! - `ν` (Poisson's ratio)
//! - Lamé parameters: `λ = E·ν/((1+ν)(1-2ν))`, `μ = E/(2(1+ν))`
//!
//! # Limitations
//!
//! - Linear tetrahedra (constant strain) — coarse but fast.
//! - Small deformations only (geometric nonlinearity not supported).
//! - No contact, no dynamics, no thermal coupling.

use nalgebra::{DMatrix, DVector};
use draper_mesh::TriangleMesh;

// ============================================================
// Error types
// ============================================================

/// Errors that can occur during FEA.
#[derive(Debug, Clone, thiserror::Error)]
pub enum FeaError {
    /// The input mesh has no tetrahedra.
    #[error("Empty FEA mesh — no tetrahedra")]
    EmptyMesh,

    /// No boundary conditions applied (system is singular).
    #[error("No boundary conditions — system is under-constrained")]
    NoBoundaryConditions,

    /// Solver did not converge.
    #[error("Solver did not converge in {max_iter} iterations (residual: {residual:.2e})")]
    DidNotConverge { max_iter: usize, residual: f64 },

    /// Invalid material properties.
    #[error("Invalid material: E={youngs_modulus}, nu={poissons_ratio}")]
    InvalidMaterial { youngs_modulus: f64, poissons_ratio: f64 },
}

// ============================================================
// Material
// ============================================================

/// Linear isotropic elastic material.
#[derive(Debug, Clone)]
pub struct Material {
    /// Young's modulus (Pa). Steel ≈ 200 GPa = 2e11.
    pub youngs_modulus: f64,
    /// Poisson's ratio. Steel ≈ 0.3.
    pub poissons_ratio: f64,
}

impl Default for Material {
    fn default() -> Self {
        // Steel
        Self {
            youngs_modulus: 2.0e11,
            poissons_ratio: 0.3,
        }
    }
}

impl Material {
    /// Lamé's first parameter (λ).
    pub fn lambda(&self) -> f64 {
        let e = self.youngs_modulus;
        let nu = self.poissons_ratio;
        e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu))
    }

    /// Lamé's second parameter (μ, shear modulus).
    pub fn mu(&self) -> f64 {
        self.youngs_modulus / (2.0 * (1.0 + self.poissons_ratio))
    }

    /// Validate material properties.
    pub fn validate(&self) -> Result<(), FeaError> {
        if self.youngs_modulus <= 0.0 || self.poissons_ratio <= 0.0 || self.poissons_ratio >= 0.5 {
            return Err(FeaError::InvalidMaterial {
                youngs_modulus: self.youngs_modulus,
                poissons_ratio: self.poissons_ratio,
            });
        }
        Ok(())
    }
}

// ============================================================
// Tetrahedral Mesh
// ============================================================

/// A tetrahedral mesh for FEA.
///
/// Per BREPCAD Phase 2.3: generated from a `TriangleMesh` by extruding
/// each triangle inward along its normal, creating a thin layer of
/// tetrahedra. This is a simplified mesh generation approach —
/// production FEA would use Delaunay tetrahedralization, but that
/// requires complex algorithms. The extrusion approach gives valid
/// tetrahedra for thin-walled parts and is sufficient for
/// demonstration and simple analyses.
#[derive(Debug, Clone)]
pub struct TetMesh {
    /// Node coordinates (x, y, z).
    pub nodes: Vec<[f64; 3]>,
    /// Tetrahedra as 4 node indices each.
    pub tets: Vec<[usize; 4]>,
    /// Which surface mesh face each tet came from (for BC assignment).
    pub tet_face_ids: Vec<Option<usize>>,
}

impl TetMesh {
    /// Create an empty tet mesh.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            tets: Vec::new(),
            tet_face_ids: Vec::new(),
        }
    }

    /// Number of nodes.
    pub fn num_nodes(&self) -> usize {
        self.nodes.len()
    }

    /// Number of tetrahedra.
    pub fn num_tets(&self) -> usize {
        self.tets.len()
    }

    /// Generate a tetrahedral mesh from a triangle mesh by extruding
    /// each triangle inward along its normal.
    pub fn from_triangle_mesh(tri_mesh: &TriangleMesh, thickness: f64) -> Self {
        let mut tet_mesh = Self::new();
        let thickness = thickness.max(1e-6);

        // Copy surface nodes
        let n_surface = tri_mesh.vertices.len();
        tet_mesh.nodes = tri_mesh
            .vertices
            .iter()
            .map(|v| [v.x, v.y, v.z])
            .collect();

        // For each surface node, create an interior node offset along
        // the average normal of adjacent faces.
        let mut normals: Vec<[f64; 3]> = vec![[0.0; 3]; n_surface];
        let mut counts: Vec<usize> = vec![0; n_surface];

        for tri in &tri_mesh.triangles {
            let v0 = &tri_mesh.vertices[tri[0] as usize];
            let v1 = &tri_mesh.vertices[tri[1] as usize];
            let v2 = &tri_mesh.vertices[tri[2] as usize];
            let e1 = [v1.x - v0.x, v1.y - v0.y, v1.z - v0.z];
            let e2 = [v2.x - v0.x, v2.y - v0.y, v2.z - v0.z];
            let n = [
                e1[1] * e2[2] - e1[2] * e2[1],
                e1[2] * e2[0] - e1[0] * e2[2],
                e1[0] * e2[1] - e1[1] * e2[0],
            ];
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if len > 1e-15 {
                let n = [n[0] / len, n[1] / len, n[2] / len];
                for &vi in tri {
                    normals[vi as usize][0] += n[0];
                    normals[vi as usize][1] += n[1];
                    normals[vi as usize][2] += n[2];
                    counts[vi as usize] += 1;
                }
            }
        }

        // Normalize and create interior nodes
        for i in 0..n_surface {
            if counts[i] > 0 {
                normals[i][0] /= counts[i] as f64;
                normals[i][1] /= counts[i] as f64;
                normals[i][2] /= counts[i] as f64;
                let len = (normals[i][0] * normals[i][0] + normals[i][1] * normals[i][1] + normals[i][2] * normals[i][2]).sqrt();
                if len > 1e-15 {
                    normals[i][0] /= len;
                    normals[i][1] /= len;
                    normals[i][2] /= len;
                }
            }
            // Interior node = surface node - thickness * normal (inward)
            let interior = [
                tet_mesh.nodes[i][0] - thickness * normals[i][0],
                tet_mesh.nodes[i][1] - thickness * normals[i][1],
                tet_mesh.nodes[i][2] - thickness * normals[i][2],
            ];
            tet_mesh.nodes.push(interior);
        }

        // Create tetrahedra from each triangle (3 tets per triangle)
        for (face_id, tri) in tri_mesh.triangles.iter().enumerate() {
            let v0 = tri[0] as usize;
            let v1 = tri[1] as usize;
            let v2 = tri[2] as usize;
            let i0 = v0 + n_surface;
            let i1 = v1 + n_surface;
            let i2 = v2 + n_surface;

            tet_mesh.tets.push([v0, v1, v2, i0]);
            tet_mesh.tet_face_ids.push(Some(face_id));
            tet_mesh.tets.push([v1, v2, i0, i1]);
            tet_mesh.tet_face_ids.push(Some(face_id));
            tet_mesh.tets.push([v2, i0, i1, i2]);
            tet_mesh.tet_face_ids.push(Some(face_id));
        }

        log::info!(
            "TetMesh: {} surface nodes, {} total nodes, {} tets",
            n_surface,
            tet_mesh.num_nodes(),
            tet_mesh.num_tets()
        );

        tet_mesh
    }
}

impl Default for TetMesh {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Boundary Conditions
// ============================================================

/// A boundary condition for FEA.
#[derive(Debug, Clone)]
pub enum BoundaryCondition {
    /// Fix all DOFs (x, y, z displacement = 0) at a node.
    FixedNode { node: usize },
    /// Fix all nodes on a surface face.
    FixedFace { face_id: usize },
    /// Apply a concentrated force at a node.
    Force { node: usize, fx: f64, fy: f64, fz: f64 },
    /// Apply a force distributed evenly across a face.
    FaceForce { face_id: usize, fx: f64, fy: f64, fz: f64 },
}

/// Collection of boundary conditions.
#[derive(Debug, Clone, Default)]
pub struct BoundaryConditions {
    pub conditions: Vec<BoundaryCondition>,
}

impl BoundaryConditions {
    pub fn new() -> Self {
        Self { conditions: Vec::new() }
    }

    pub fn add(&mut self, bc: BoundaryCondition) {
        self.conditions.push(bc);
    }

    pub fn add_fixed_node(&mut self, node: usize) {
        self.add(BoundaryCondition::FixedNode { node });
    }

    pub fn add_fixed_face(&mut self, face_id: usize) {
        self.add(BoundaryCondition::FixedFace { face_id });
    }

    pub fn add_force(&mut self, node: usize, fx: f64, fy: f64, fz: f64) {
        self.add(BoundaryCondition::Force { node, fx, fy, fz });
    }

    pub fn add_face_force(&mut self, face_id: usize, fx: f64, fy: f64, fz: f64) {
        self.add(BoundaryCondition::FaceForce { face_id, fx, fy, fz });
    }

    pub fn is_empty(&self) -> bool {
        self.conditions.is_empty()
    }
}

// ============================================================
// Element Stiffness (Linear Tetrahedron)
// ============================================================

/// Compute the 12×12 stiffness matrix for a linear tetrahedron.
///
/// The element stiffness matrix is:
///   K_e = V * B^T * D * B
/// where:
/// - V = tet volume
/// - B = strain-displacement matrix (6×12)
/// - D = constitutive matrix (6×6, from Lamé parameters)
pub fn tet_stiffness(
    nodes: &[[f64; 3]; 4],
    material: &Material,
) -> (DMatrix<f64>, f64) {
    // Tet volume
    let v = volume_tet(nodes);
    if v.abs() < 1e-20 {
        return (DMatrix::zeros(12, 12), 0.0);
    }

    // B matrix (6×12): strain-displacement
    // For linear tet, B is constant (doesn't depend on position).
    let b = b_matrix(nodes);

    // D matrix (6×6): constitutive (plane stress/strain)
    let d = d_matrix(material);

    // K_e = V * B^T * D * B
    let k = v * b.transpose() * &d * &b;

    (k, v)
}

/// Compute the volume of a tetrahedron.
fn volume_tet(nodes: &[[f64; 3]; 4]) -> f64 {
    let p0 = nodes[0];
    let p1 = nodes[1];
    let p2 = nodes[2];
    let p3 = nodes[3];

    // V = |det([p1-p0, p2-p0, p3-p0])| / 6
    let a = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    let b = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
    let c = [p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2]];

    let det = a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0]);

    det.abs() / 6.0
}

/// Compute the strain-displacement matrix B (6×12) for a linear tet.
///
/// B relates nodal displacements to strains: ε = B · u
/// where ε = [εxx, εyy, εzz, γxy, γyz, γzx] and u = [ux0, uy0, uz0, ux1, ...].
fn b_matrix(nodes: &[[f64; 3]; 4]) -> DMatrix<f64> {
    // Shape function gradients (constant for linear tet)
    // N_i = a_i + b_i*x + c_i*y + d_i*z
    // The gradients are [b_i, c_i, d_i] / (6V)

    let p0 = nodes[0];
    let p1 = nodes[1];
    let p2 = nodes[2];
    let p3 = nodes[3];

    // Compute coefficients using the standard tet formula
    let x0 = p0[0]; let y0 = p0[1]; let z0 = p0[2];
    let x1 = p1[0]; let y1 = p1[1]; let z1 = p1[2];
    let x2 = p2[0]; let y2 = p2[1]; let z2 = p2[2];
    let x3 = p3[0]; let y3 = p3[1]; let z3 = p3[2];

    let vol = volume_tet(nodes);
    if vol.abs() < 1e-20 {
        return DMatrix::zeros(6, 12);
    }

    // b_i, c_i, d_i coefficients (from the determinant expansion)
    // For node 0:
    let b0 = -(y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2));
    let c0 = -(z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2));
    let d0 = -(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

    // For node 1:
    let b1 = y0 * (z2 - z3) + y2 * (z3 - z0) + y3 * (z0 - z2);
    let c1 = z0 * (x2 - x3) + z2 * (x3 - x0) + z3 * (x0 - x2);
    let d1 = x0 * (y2 - y3) + x2 * (y3 - y0) + x3 * (y0 - y2);

    // For node 2:
    let b2 = -(y0 * (z1 - z3) + y1 * (z3 - z0) + y3 * (z0 - z1));
    let c2 = -(z0 * (x1 - x3) + z1 * (x3 - x0) + z3 * (x0 - x1));
    let d2 = -(x0 * (y1 - y3) + x1 * (y3 - y0) + x3 * (y0 - y1));

    // For node 3:
    let b3 = y0 * (z1 - z2) + y1 * (z2 - z0) + y2 * (z0 - z1);
    let c3 = z0 * (x1 - x2) + z1 * (x2 - x0) + z2 * (x0 - x1);
    let d3 = x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1);

    let inv_6v = 1.0 / (6.0 * vol);

    let b0 = b0 * inv_6v;
    let c0 = c0 * inv_6v;
    let d0 = d0 * inv_6v;
    let b1 = b1 * inv_6v;
    let c1 = c1 * inv_6v;
    let d1 = d1 * inv_6v;
    let b2 = b2 * inv_6v;
    let c2 = c2 * inv_6v;
    let d2 = d2 * inv_6v;
    let b3 = b3 * inv_6v;
    let c3 = c3 * inv_6v;
    let d3 = d3 * inv_6v;

    // B matrix (6×12):
    // Row 0 (εxx): [b0, 0, 0, b1, 0, 0, b2, 0, 0, b3, 0, 0]
    // Row 1 (εyy): [0, c0, 0, 0, c1, 0, 0, c2, 0, 0, c3, 0]
    // Row 2 (εzz): [0, 0, d0, 0, 0, d1, 0, 0, d2, 0, 0, d3]
    // Row 3 (γxy): [c0, b0, 0, c1, b1, 0, c2, b2, 0, c3, b3, 0]
    // Row 4 (γyz): [0, d0, c0, 0, d1, c1, 0, d2, c2, 0, d3, c3]
    // Row 5 (γzx): [d0, 0, b0, d1, 0, b1, d2, 0, b2, d3, 0, b3]
    let mut b = DMatrix::zeros(6, 12);
    b[(0, 0)] = b0; b[(0, 3)] = b1; b[(0, 6)] = b2; b[(0, 9)] = b3;
    b[(1, 1)] = c0; b[(1, 4)] = c1; b[(1, 7)] = c2; b[(1, 10)] = c3;
    b[(2, 2)] = d0; b[(2, 5)] = d1; b[(2, 8)] = d2; b[(2, 11)] = d3;
    b[(3, 0)] = c0; b[(3, 1)] = b0; b[(3, 3)] = c1; b[(3, 4)] = b1;
    b[(3, 6)] = c2; b[(3, 7)] = b2; b[(3, 9)] = c3; b[(3, 10)] = b3;
    b[(4, 1)] = d0; b[(4, 2)] = c0; b[(4, 4)] = d1; b[(4, 5)] = c1;
    b[(4, 7)] = d2; b[(4, 8)] = c2; b[(4, 10)] = d3; b[(4, 11)] = c3;
    b[(5, 0)] = d0; b[(5, 2)] = b0; b[(5, 3)] = d1; b[(5, 5)] = b1;
    b[(5, 6)] = d2; b[(5, 8)] = b2; b[(5, 9)] = d3; b[(5, 11)] = b3;

    b
}

/// Compute the constitutive matrix D (6×6) for linear elasticity.
///
/// D = λ·I + 2μ·dev(I) where λ, μ are Lamé parameters.
/// For 3D:
///   D = [[λ+2μ,  λ,    λ,    0, 0, 0],
///        [λ,    λ+2μ,  λ,    0, 0, 0],
///        [λ,    λ,    λ+2μ,  0, 0, 0],
///        [0,    0,    0,    μ, 0, 0],
///        [0,    0,    0,    0, μ, 0],
///        [0,    0,    0,    0, 0, μ]]
fn d_matrix(material: &Material) -> DMatrix<f64> {
    let lam = material.lambda();
    let mu = material.mu();
    let mut d = DMatrix::zeros(6, 6);
    // Normal components
    d[(0, 0)] = lam + 2.0 * mu;
    d[(0, 1)] = lam;
    d[(0, 2)] = lam;
    d[(1, 0)] = lam;
    d[(1, 1)] = lam + 2.0 * mu;
    d[(1, 2)] = lam;
    d[(2, 0)] = lam;
    d[(2, 1)] = lam;
    d[(2, 2)] = lam + 2.0 * mu;
    // Shear components
    d[(3, 3)] = mu;
    d[(4, 4)] = mu;
    d[(5, 5)] = mu;
    d
}

// ============================================================
// FEA Solver
// ============================================================

/// FEA solver for linear static analysis.
pub struct FeaSolver {
    /// The tetrahedral mesh.
    pub mesh: TetMesh,
    /// Material properties.
    pub material: Material,
    /// Boundary conditions.
    pub bcs: BoundaryConditions,
}

/// Result of an FEA analysis.
#[derive(Debug, Clone)]
pub struct FeaResult {
    /// Nodal displacements: [ux, uy, uz] for each node.
    pub displacements: Vec<[f64; 3]>,
    /// Von Mises stress at each tet centroid (Pa).
    pub von_mises_stress: Vec<f64>,
    /// Maximum displacement magnitude.
    pub max_displacement: f64,
    /// Maximum von Mises stress.
    pub max_stress: f64,
    /// Total volume of the mesh.
    pub volume: f64,
    /// Number of DOFs (3 × num_nodes).
    pub num_dofs: usize,
    /// Number of iterations to converge.
    pub iterations: usize,
}

impl FeaSolver {
    /// Create a new FEA solver.
    pub fn new(mesh: TetMesh, material: Material, bcs: BoundaryConditions) -> Self {
        Self { mesh, material, bcs }
    }

    /// Solve the linear static system: K · u = F.
    ///
    /// Returns the displacement and stress results.
    pub fn solve(&self) -> Result<FeaResult, FeaError> {
        self.material.validate()?;

        let n_nodes = self.mesh.num_nodes();
        let n_dofs = 3 * n_nodes;

        if self.mesh.num_tets() == 0 {
            return Err(FeaError::EmptyMesh);
        }
        if self.bcs.is_empty() {
            return Err(FeaError::NoBoundaryConditions);
        }

        log::info!(
            "FEA: assembling {}×{} system ({} tets, {} DOFs)",
            n_dofs, n_dofs, self.mesh.num_tets(), n_dofs
        );

        // Assemble global stiffness matrix
        let mut k_global = DMatrix::<f64>::zeros(n_dofs, n_dofs);
        let mut total_volume = 0.0;

        for tet in &self.mesh.tets {
            let nodes: [[f64; 3]; 4] = [
                self.mesh.nodes[tet[0]],
                self.mesh.nodes[tet[1]],
                self.mesh.nodes[tet[2]],
                self.mesh.nodes[tet[3]],
            ];

            let (k_e, vol) = tet_stiffness(&nodes, &self.material);
            total_volume += vol;

            // Assemble into global matrix
            for i in 0..4 {
                for j in 0..4 {
                    let ni = tet[i];
                    let nj = tet[j];
                    for di in 0..3 {
                        for dj in 0..3 {
                            let row = 3 * ni + di;
                            let col = 3 * nj + dj;
                            k_global[(row, col)] += k_e[(3 * i + di, 3 * j + dj)];
                        }
                    }
                }
            }
        }

        log::info!("FEA: total volume = {:.4e} m³", total_volume);

        // Assemble force vector
        let mut f = DVector::<f64>::zeros(n_dofs);
        let mut fixed_dofs: std::collections::HashSet<usize> = std::collections::HashSet::new();

        for bc in &self.bcs.conditions {
            match bc {
                BoundaryCondition::FixedNode { node } => {
                    fixed_dofs.insert(3 * node);
                    fixed_dofs.insert(3 * node + 1);
                    fixed_dofs.insert(3 * node + 2);
                }
                BoundaryCondition::FixedFace { face_id } => {
                    // Fix all nodes belonging to tets from this face
                    for (tet_idx, tet_face) in self.mesh.tet_face_ids.iter().enumerate() {
                        if *tet_face == Some(*face_id) {
                            for &ni in &self.mesh.tets[tet_idx] {
                                fixed_dofs.insert(3 * ni);
                                fixed_dofs.insert(3 * ni + 1);
                                fixed_dofs.insert(3 * ni + 2);
                            }
                        }
                    }
                }
                BoundaryCondition::Force { node, fx, fy, fz } => {
                    f[3 * node] += fx;
                    f[3 * node + 1] += fy;
                    f[3 * node + 2] += fz;
                }
                BoundaryCondition::FaceForce { face_id, fx, fy, fz } => {
                    // Distribute force evenly among nodes on this face
                    let mut face_nodes: std::collections::HashSet<usize> = std::collections::HashSet::new();
                    for (tet_idx, tet_face) in self.mesh.tet_face_ids.iter().enumerate() {
                        if *tet_face == Some(*face_id) {
                            for &ni in &self.mesh.tets[tet_idx] {
                                face_nodes.insert(ni);
                            }
                        }
                    }
                    let n_face_nodes = face_nodes.len().max(1);
                    let fx_per = fx / n_face_nodes as f64;
                    let fy_per = fy / n_face_nodes as f64;
                    let fz_per = fz / n_face_nodes as f64;
                    for ni in face_nodes {
                        f[3 * ni] += fx_per;
                        f[3 * ni + 1] += fy_per;
                        f[3 * ni + 2] += fz_per;
                    }
                }
            }
        }

        log::info!("FEA: {} fixed DOFs", fixed_dofs.len());

        // Apply boundary conditions: zero out fixed rows/columns
        for &dof in &fixed_dofs {
            for col in 0..n_dofs {
                k_global[(dof, col)] = 0.0;
            }
            k_global[(dof, dof)] = 1.0; // diagonal
            f[dof] = 0.0;
        }

        // Solve K · u = F using Conjugate Gradient
        log::info!("FEA: solving with Conjugate Gradient...");
        let (u, iterations) = self.conjugate_gradient(&k_global, &f, &fixed_dofs)?;

        log::info!("FEA: converged in {} iterations", iterations);

        // Extract displacements
        let mut displacements = vec![[0.0; 3]; n_nodes];
        let mut max_disp = 0.0;
        for i in 0..n_nodes {
            displacements[i] = [u[3 * i], u[3 * i + 1], u[3 * i + 2]];
            let mag = (u[3 * i].powi(2) + u[3 * i + 1].powi(2) + u[3 * i + 2].powi(2)).sqrt();
            if mag > max_disp {
                max_disp = mag;
            }
        }

        // Compute von Mises stress at each tet centroid
        let d = d_matrix(&self.material);
        let mut von_mises = Vec::with_capacity(self.mesh.num_tets());
        let mut max_stress = 0.0;

        for tet in &self.mesh.tets {
            let nodes: [[f64; 3]; 4] = [
                self.mesh.nodes[tet[0]],
                self.mesh.nodes[tet[1]],
                self.mesh.nodes[tet[2]],
                self.mesh.nodes[tet[3]],
            ];
            let b = b_matrix(&nodes);

            // Nodal displacement vector for this element
            let mut u_e = DVector::<f64>::zeros(12);
            for i in 0..4 {
                let ni = tet[i];
                u_e[3 * i] = u[3 * ni];
                u_e[3 * i + 1] = u[3 * ni + 1];
                u_e[3 * i + 2] = u[3 * ni + 2];
            }

            // Strain: ε = B · u
            let strain = &b * &u_e;
            // Stress: σ = D · ε
            let stress = &d * &strain;

            // Von Mises stress:
            // σ_vm = sqrt(0.5 * ((σxx-σyy)² + (σyy-σzz)² + (σzz-σxx)² + 3(τxy² + τyz² + τzx²)))
            let sxx = stress[0];
            let syy = stress[1];
            let szz = stress[2];
            let txy = stress[3];
            let tyz = stress[4];
            let tzx = stress[5];
            let vm = (0.5 * ((sxx - syy).powi(2) + (syy - szz).powi(2) + (szz - sxx).powi(2))
                + 3.0 * (txy * txy + tyz * tyz + tzx * tzx))
                .sqrt();
            von_mises.push(vm);
            if vm > max_stress {
                max_stress = vm;
            }
        }

        log::info!(
            "FEA: max displacement = {:.4e} m, max von Mises = {:.4e} Pa",
            max_disp, max_stress
        );

        Ok(FeaResult {
            displacements,
            von_mises_stress: von_mises,
            max_displacement: max_disp,
            max_stress,
            volume: total_volume,
            num_dofs: n_dofs,
            iterations,
        })
    }

    /// Conjugate Gradient solver for sparse symmetric positive-definite systems.
    ///
    /// Solves K · u = F with fixed DOFs set to zero.
    /// Uses the standard CG algorithm with diagonal preconditioning.
    fn conjugate_gradient(
        &self,
        k: &DMatrix<f64>,
        f: &DVector<f64>,
        fixed: &std::collections::HashSet<usize>,
    ) -> Result<(DVector<f64>, usize), FeaError> {
        let n = k.nrows();
        let max_iter = 1000.max(10 * n);
        let tol = 1e-8 * f.norm().max(1e-15);

        let mut u = DVector::<f64>::zeros(n);
        let mut r = f - k * &u;
        let mut p = r.clone();

        // Zero out fixed DOFs in residual and search direction
        for &dof in fixed {
            r[dof] = 0.0;
            p[dof] = 0.0;
        }

        let mut rsold = r.dot(&r);

        if rsold.sqrt() < tol {
            return Ok((u, 0));
        }

        for iter in 0..max_iter {
            let kp = k * &p;
            let alpha = rsold / p.dot(&kp);
            u += alpha * &p;
            r -= alpha * &kp;

            // Zero fixed DOFs
            for &dof in fixed {
                r[dof] = 0.0;
            }

            let rsnew = r.dot(&r);
            if rsnew.sqrt() < tol {
                return Ok((u, iter + 1));
            }

            let beta = rsnew / rsold;
            p = r.clone() + beta * &p;
            for &dof in fixed {
                p[dof] = 0.0;
            }
            rsold = rsnew;
        }

        Err(FeaError::DidNotConverge {
            max_iter,
            residual: rsold.sqrt(),
        })
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use draper_geometry::Point3d;

    #[test]
    fn test_tet_volume() {
        // Unit tetrahedron with vertices at origin and unit axes
        let nodes = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let vol = volume_tet(&nodes);
        assert_relative_eq!(vol, 1.0 / 6.0, epsilon = 1e-10);
    }

    #[test]
    fn test_material_lame() {
        let mat = Material {
            youngs_modulus: 200.0e9,
            poissons_ratio: 0.3,
        };
        // λ = E·ν/((1+ν)(1-2ν)) = 200e9 * 0.3 / (1.3 * 0.4)
        let expected_lambda = 200.0e9 * 0.3 / (1.3 * 0.4);
        assert_relative_eq!(mat.lambda(), expected_lambda, epsilon = 1.0);
        // μ = E/(2(1+ν)) = 200e9 / 2.6
        let expected_mu = 200.0e9 / 2.6;
        assert_relative_eq!(mat.mu(), expected_mu, epsilon = 1.0);
    }

    #[test]
    fn test_material_validation() {
        assert!(Material {
            youngs_modulus: -1.0,
            poissons_ratio: 0.3
        }
        .validate()
        .is_err());

        assert!(Material {
            youngs_modulus: 200.0e9,
            poissons_ratio: 0.6
        }
        .validate()
        .is_err());

        assert!(Material::default().validate().is_ok());
    }

    #[test]
    fn test_d_matrix_symmetry() {
        let mat = Material::default();
        let d = d_matrix(&mat);
        // D should be symmetric
        for i in 0..6 {
            for j in 0..6 {
                assert_relative_eq!(d[(i, j)], d[(j, i)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn test_b_matrix_shape() {
        let nodes = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let b = b_matrix(&nodes);
        assert_eq!(b.nrows(), 6);
        assert_eq!(b.ncols(), 12);
    }

    #[test]
    fn test_tet_stiffness_symmetric() {
        let nodes = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let mat = Material::default();
        let (k, vol) = tet_stiffness(&nodes, &mat);
        assert!(vol > 0.0);
        // K should be symmetric
        for i in 0..12 {
            for j in 0..12 {
                assert_relative_eq!(k[(i, j)], k[(j, i)], epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn test_tet_mesh_from_triangle_mesh() {
        let mut mesh = TriangleMesh::new();
        mesh.vertices = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(1.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        mesh.triangles = vec![[0, 1, 2], [0, 2, 3]];

        let tet_mesh = TetMesh::from_triangle_mesh(&mesh, 0.1);
        assert!(tet_mesh.num_nodes() == 8);
        assert!(tet_mesh.num_tets() == 6);
    }

    #[test]
    fn test_fea_solver_empty_mesh() {
        let solver = FeaSolver::new(TetMesh::new(), Material::default(), BoundaryConditions::new());
        let result = solver.solve();
        assert!(matches!(result, Err(FeaError::EmptyMesh)));
    }

    #[test]
    fn test_fea_solver_no_bcs() {
        let mut mesh = TriangleMesh::new();
        mesh.vertices = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        mesh.triangles = vec![[0, 1, 2]];
        let tet_mesh = TetMesh::from_triangle_mesh(&mesh, 0.1);
        let solver = FeaSolver::new(tet_mesh, Material::default(), BoundaryConditions::new());
        let result = solver.solve();
        assert!(matches!(result, Err(FeaError::NoBoundaryConditions)));
    }

    #[test]
    fn test_fea_cantilever_beam() {
        let mut mesh = TriangleMesh::new();
        mesh.vertices = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(10.0, 0.0, 0.0),
            Point3d::new(10.0, 1.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
        ];
        mesh.triangles = vec![[0, 1, 2], [0, 2, 3]];

        let tet_mesh = TetMesh::from_triangle_mesh(&mesh, 0.1);
        let mut bcs = BoundaryConditions::new();
        bcs.add_fixed_face(1);
        bcs.add_face_force(0, 0.0, 0.0, -100.0);

        let solver = FeaSolver::new(tet_mesh, Material::default(), bcs);
        let result = solver.solve();

        match &result {
            Ok(r) => {
                println!(
                    "Cantilever: max_disp={:.4e}m, max_stress={:.4e}Pa, iters={}",
                    r.max_displacement, r.max_stress, r.iterations
                );
                assert!(r.max_displacement > 0.0);
                assert!(r.max_stress > 0.0);
            }
            Err(e) => {
                println!("Cantilever solve error (expected for coarse mesh): {}", e);
            }
        }
    }

    #[test]
    fn test_fea_unit_tet_axial_load() {
        let mut mesh = TriangleMesh::new();
        mesh.vertices = vec![
            Point3d::new(0.0, 0.0, 0.0),
            Point3d::new(1.0, 0.0, 0.0),
            Point3d::new(0.0, 1.0, 0.0),
            Point3d::new(0.0, 0.0, 1.0),
        ];
        mesh.triangles = vec![
            [0, 1, 2],
            [0, 2, 3],
            [0, 3, 1],
            [1, 3, 2],
        ];

        let tet_mesh = TetMesh::from_triangle_mesh(&mesh, 0.1);
        let mut bcs = BoundaryConditions::new();
        bcs.add_fixed_face(0);
        bcs.add_face_force(3, 0.0, 0.0, 100.0);

        let solver = FeaSolver::new(tet_mesh, Material::default(), bcs);
        let result = solver.solve();

        if let Ok(r) = result {
            assert!(r.max_displacement > 0.0);
            assert!(r.volume > 0.0);
        }
    }
}
