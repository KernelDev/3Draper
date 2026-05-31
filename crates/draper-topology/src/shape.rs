// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Shape storage — efficient storage and indexing of topological entities.

use crate::entity::*;
use std::collections::HashMap;

/// A shape storage that owns all topological entities and provides fast lookup.
#[derive(Clone, Debug, Default)]
pub struct ShapeStorage {
    pub vertices: HashMap<TopoId, Vertex>,
    pub edges: HashMap<TopoId, Edge>,
    pub coedges: HashMap<TopoId, CoEdge>,
    pub wires: HashMap<TopoId, Wire>,
    pub faces: HashMap<TopoId, Face>,
    pub shells: HashMap<TopoId, Shell>,
    pub solids: HashMap<TopoId, Solid>,
}

impl ShapeStorage {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_vertex(&mut self, v: Vertex) -> TopoId {
        let id = v.id;
        self.vertices.insert(id, v);
        id
    }

    pub fn add_edge(&mut self, e: Edge) -> TopoId {
        let id = e.id;
        self.edges.insert(id, e);
        id
    }

    pub fn add_face(&mut self, f: Face) -> TopoId {
        let id = f.id;
        self.faces.insert(id, f);
        id
    }

    pub fn add_shell(&mut self, s: Shell) -> TopoId {
        let id = s.id;
        self.shells.insert(id, s);
        id
    }

    pub fn add_solid(&mut self, s: Solid) -> TopoId {
        let id = s.id;
        self.solids.insert(id, s);
        id
    }

    /// Get a vertex by ID.
    pub fn get_vertex(&self, id: &TopoId) -> Option<&Vertex> {
        self.vertices.get(id)
    }

    /// Get an edge by ID.
    pub fn get_edge(&self, id: &TopoId) -> Option<&Edge> {
        self.edges.get(id)
    }

    /// Get a face by ID.
    pub fn get_face(&self, id: &TopoId) -> Option<&Face> {
        self.faces.get(id)
    }

    /// Number of faces.
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// Number of edges.
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }
}
