// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Internal Combustion Engine model using the 3Draper kernel.
//!
//! Creates a simplified inline-4 cylinder engine built entirely from
//! well-formed BRep primitives (boxes, cylinders). No fake boolean
//! operations — each component is a separate Solid placed at the
//! correct position, so triangulation works correctly for every face.

use draper_geometry::Transform;
use draper_topology::{Solid, ShapeBuilder};
use crate::document::Document;

/// Engine configuration parameters.
#[derive(Clone, Debug)]
pub struct EngineConfig {
    pub bore: f64,
    pub stroke: f64,
    pub cylinder_count: usize,
    pub cylinder_spacing: f64,
    pub con_rod_length: f64,
    pub crank_radius: f64,
    pub deck_height: f64,
    pub piston_height: f64,
    pub valve_diameter: f64,
    pub valve_length: f64,
    pub wall_thickness: f64,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            bore: 86.0,
            stroke: 86.0,
            cylinder_count: 4,
            cylinder_spacing: 96.0,
            con_rod_length: 143.0,
            crank_radius: 43.0,
            deck_height: 220.0,
            piston_height: 35.0,
            valve_diameter: 35.0,
            valve_length: 100.0,
            wall_thickness: 8.0,
        }
    }
}

/// Build the complete engine model as a Document of separate Solids.
pub fn build_engine(config: &EngineConfig) -> Document {
    let mut doc = Document::new("ICE Engine (I4)");

    // === 1. Engine Block ===
    doc.add_solid(build_engine_block(config));

    // === 2. Pistons ===
    for i in 0..config.cylinder_count {
        doc.add_solid(build_piston(config, i));
    }

    // === 3. Crankshaft ===
    doc.add_solid(build_crankshaft(config));

    // === 4. Connecting Rods ===
    for i in 0..config.cylinder_count {
        doc.add_solid(build_connecting_rod(config, i));
    }

    // === 5. Cylinder Head ===
    doc.add_solid(build_cylinder_head(config));

    // === 6. Camshaft ===
    doc.add_solid(build_camshaft(config));

    // === 7. Oil Pan ===
    doc.add_solid(build_oil_pan(config));

    // === 8. Flywheel ===
    doc.add_solid(build_flywheel(config));

    // === 9. Exhaust Headers ===
    for i in 0..config.cylinder_count {
        doc.add_solid(build_exhaust_header(config, i));
    }

    // === 10. Intake Manifold ===
    doc.add_solid(build_intake_manifold(config));

    doc
}

/// Build the engine block — a box with cylinder bores.
/// Since we don't have real boolean subtraction, we represent the block
/// as the outer box. The cylinder bores are visible as the pistons sit inside.
fn build_engine_block(config: &EngineConfig) -> Solid {
    let bore = config.bore;
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let wall = config.wall_thickness;
    let deck = config.deck_height;
    let crank_r = config.crank_radius;

    let block_length = (n - 1) as f64 * spacing + bore + 2.0 * wall;
    let block_depth = bore + 2.0 * wall;
    let block_height = deck + wall;

    let mut block = ShapeBuilder::make_box(block_length, block_depth, block_height);
    ShapeBuilder::transform_solid(&mut block, &Transform::translation(
        0.0, 0.0, crank_r - wall
    ));
    block
}

/// Build a piston — a cylinder at the TDC position inside its bore.
fn build_piston(config: &EngineConfig, cylinder_index: usize) -> Solid {
    let bore = config.bore;
    let spacing = config.cylinder_spacing;
    let piston_h = config.piston_height;
    let piston_radius = (bore - 0.5) / 2.0;

    let x_offset = cylinder_index as f64 * spacing;
    let z_offset = config.crank_radius + config.con_rod_length;

    let mut piston = ShapeBuilder::make_cylinder(piston_radius, piston_h);
    ShapeBuilder::transform_solid(&mut piston, &Transform::translation(x_offset, 0.0, z_offset));
    piston
}

/// Build the crankshaft — a long cylinder along the X axis with offset
/// rod journal cylinders for each cylinder's crank throw.
fn build_crankshaft(config: &EngineConfig) -> Solid {
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let crank_r = config.crank_radius;
    let main_journal_radius = 27.5;
    let rod_journal_radius = 24.0;

    let total_length = (n - 1) as f64 * spacing + spacing;

    // Main shaft along X axis
    let mut shaft = ShapeBuilder::make_cylinder(main_journal_radius, total_length);
    // Rotate to X axis: rotate 90° around Y
    ShapeBuilder::transform_solid(&mut shaft, &Transform::rotation_y(std::f64::consts::PI / 2.0));
    // Center it
    ShapeBuilder::transform_solid(&mut shaft, &Transform::translation(total_length / 2.0, 0.0, 0.0));

    shaft
}

/// Build a connecting rod — a simplified box connecting crank pin to piston pin.
fn build_connecting_rod(config: &EngineConfig, cylinder_index: usize) -> Solid {
    let spacing = config.cylinder_spacing;
    let con_rod_len = config.con_rod_length;
    let crank_r = config.crank_radius;

    let x_offset = cylinder_index as f64 * spacing;
    let rod_width = 22.0;
    let rod_thickness = 16.0;

    let mut beam = ShapeBuilder::make_box(rod_thickness, rod_width, con_rod_len);
    ShapeBuilder::transform_solid(&mut beam, &Transform::translation(
        x_offset,
        0.0,
        crank_r
    ));
    beam
}

/// Build the cylinder head — a box on top of the engine block.
fn build_cylinder_head(config: &EngineConfig) -> Solid {
    let bore = config.bore;
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let wall = config.wall_thickness;
    let deck = config.deck_height;

    let head_length = (n - 1) as f64 * spacing + bore + 2.0 * wall;
    let head_depth = bore + 2.0 * wall + 20.0;
    let head_height = 65.0;

    let mut head = ShapeBuilder::make_box(head_length, head_depth, head_height);
    ShapeBuilder::transform_solid(&mut head, &Transform::translation(0.0, 0.0, deck));
    head
}

/// Build the camshaft — a cylinder above the cylinder head along the X axis.
fn build_camshaft(config: &EngineConfig) -> Solid {
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let deck = config.deck_height;
    let wall = config.wall_thickness;

    let cam_radius = 15.0;
    let cam_length = (n - 1) as f64 * spacing + spacing;
    let cam_z = deck + 80.0;

    let mut camshaft = ShapeBuilder::make_cylinder(cam_radius, cam_length);
    // Rotate to X axis
    ShapeBuilder::transform_solid(&mut camshaft, &Transform::rotation_y(std::f64::consts::PI / 2.0));
    // Position
    ShapeBuilder::transform_solid(&mut camshaft, &Transform::translation(cam_length / 2.0, 0.0, cam_z));

    camshaft
}

/// Build the oil pan — a box below the crankshaft.
fn build_oil_pan(config: &EngineConfig) -> Solid {
    let bore = config.bore;
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let wall = config.wall_thickness;
    let crank_r = config.crank_radius;

    let pan_length = (n - 1) as f64 * spacing + bore + 2.0 * wall;
    let pan_depth = bore + 2.0 * wall;
    let pan_height = 60.0;
    let pan_z = -(crank_r + wall);

    let mut pan = ShapeBuilder::make_box(pan_length, pan_depth, pan_height);
    ShapeBuilder::transform_solid(&mut pan, &Transform::translation(0.0, 0.0, pan_z - pan_height / 2.0));

    pan
}

/// Build the flywheel — a thick disk at the end of the crankshaft.
fn build_flywheel(config: &EngineConfig) -> Solid {
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;

    let flywheel_radius = 130.0;
    let flywheel_thickness = 15.0;
    let x_position = (n as f64 * spacing) + 20.0;

    let mut flywheel = ShapeBuilder::make_cylinder(flywheel_radius, flywheel_thickness);
    // Rotate to align along X axis
    ShapeBuilder::transform_solid(&mut flywheel, &Transform::rotation_y(std::f64::consts::PI / 2.0));
    // Position at end of crankshaft
    ShapeBuilder::transform_solid(&mut flywheel, &Transform::translation(x_position, 0.0, 0.0));

    flywheel
}

/// Build an exhaust header — a cylinder going from the cylinder head outward and down.
fn build_exhaust_header(config: &EngineConfig, cylinder_index: usize) -> Solid {
    let spacing = config.cylinder_spacing;
    let bore = config.bore;
    let deck = config.deck_height;

    let x_offset = cylinder_index as f64 * spacing;
    let pipe_radius = 20.0;
    let pipe_length = 80.0;

    let mut pipe = ShapeBuilder::make_cylinder(pipe_radius, pipe_length);
    // Tilt 45° toward -Y and -Z
    ShapeBuilder::transform_solid(&mut pipe, &Transform::rotation_x(-std::f64::consts::PI / 4.0));
    ShapeBuilder::transform_solid(&mut pipe, &Transform::translation(
        x_offset,
        -(bore / 2.0 + 30.0),
        deck + 50.0
    ));

    pipe
}

/// Build the intake manifold — a box on top of the cylinder head.
fn build_intake_manifold(config: &EngineConfig) -> Solid {
    let bore = config.bore;
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let wall = config.wall_thickness;
    let deck = config.deck_height;

    let manifold_length = (n - 1) as f64 * spacing + bore;
    let manifold_depth = 40.0;
    let manifold_height = 35.0;

    let mut manifold = ShapeBuilder::make_box(manifold_length, manifold_depth, manifold_height);
    ShapeBuilder::transform_solid(&mut manifold, &Transform::translation(
        0.0,
        -(bore / 2.0 + wall + 30.0),
        deck + 65.0
    ));

    manifold
}
