// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Document — top-level container for a CAD model.

use draper_topology::{Solid, Compound};
use draper_mesh::{TriangleMesh, triangulate_compound, TriangulationParams};

/// A CAD document containing one or more solids.
#[derive(Clone, Debug)]
pub struct Document {
    /// Name of the document.
    pub name: String,
    /// The root compound (assembly).
    pub root: Compound,
    /// Triangulation parameters.
    pub tri_params: TriangulationParams,
}

impl Document {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            root: Compound::new(),
            tri_params: TriangulationParams::default(),
        }
    }

    /// Add a solid to the document.
    pub fn add_solid(&mut self, solid: Solid) {
        self.root.add_solid(solid);
    }

    /// Triangulate the entire document.
    pub fn triangulate(&self) -> TriangleMesh {
        triangulate_compound(&self.root, &self.tri_params)
    }

    /// Get all solids.
    pub fn solids(&self) -> Vec<&Solid> {
        let mut result = Vec::new();
        result.extend(&self.root.solids);
        for sub in &self.root.compounds {
            result.extend(&sub.solids);
        }
        result
    }

    /// Number of solids.
    pub fn solid_count(&self) -> usize {
        self.root.solids.len()
    }
}
