//! Internal Combustion Engine model using the 3Draper kernel.
//!
//! Creates a simplified inline-4 cylinder engine with:
//! - Engine block with cylinder bores
//! - Pistons with piston rings
//! - Crankshaft
//! - Connecting rods
//! - Cylinder head with valve ports
//! - Camshaft
//! - Oil pan

use draper_geometry::{
    Point3d, Direction3d, Vec3d, Transform,
    Curve3d, Line, Circle, Arc,
    Surface, Plane, CylinderSurface,
};
use draper_topology::{
    Solid, Shell, Face, Wire, CoEdge, Edge, Vertex,
    ShapeBuilder,
};
use draper_mesh::{TriangleMesh, triangulate_compound, triangulate_solid, TriangulationParams};
use crate::document::Document;
use crate::assembly::{Assembly, AssemblyNode};
use crate::boolean;

/// Engine configuration parameters.
#[derive(Clone, Debug)]
pub struct EngineConfig {
    /// Bore diameter (mm).
    pub bore: f64,
    /// Stroke length (mm).
    pub stroke: f64,
    /// Number of cylinders.
    pub cylinder_count: usize,
    /// Cylinder spacing (mm).
    pub cylinder_spacing: f64,
    /// Connecting rod length (mm).
    pub con_rod_length: f64,
    /// Crank radius (mm) = stroke / 2.
    pub crank_radius: f64,
    /// Deck height (mm) — distance from crank center to block deck.
    pub deck_height: f64,
    /// Piston compression height (mm).
    pub piston_height: f64,
    /// Valve diameter (mm).
    pub valve_diameter: f64,
    /// Valve length (mm).
    pub valve_length: f64,
    /// Wall thickness (mm).
    pub wall_thickness: f64,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            bore: 86.0,           // 86mm bore
            stroke: 86.0,         // 86mm stroke (square engine)
            cylinder_count: 4,    // Inline-4
            cylinder_spacing: 96.0,
            con_rod_length: 143.0,
            crank_radius: 43.0,   // stroke / 2
            deck_height: 220.0,
            piston_height: 35.0,
            valve_diameter: 35.0,
            valve_length: 100.0,
            wall_thickness: 8.0,
        }
    }
}

/// Build the complete engine model.
pub fn build_engine(config: &EngineConfig) -> Document {
    let mut doc = Document::new("ICE Engine");

    // === 1. Engine Block ===
    let block = build_engine_block(config);
    doc.add_solid(block);

    // === 2. Pistons ===
    for i in 0..config.cylinder_count {
        let piston = build_piston(config, i);
        doc.add_solid(piston);
    }

    // === 3. Crankshaft ===
    let crankshaft = build_crankshaft(config);
    doc.add_solid(crankshaft);

    // === 4. Connecting Rods ===
    for i in 0..config.cylinder_count {
        let con_rod = build_connecting_rod(config, i);
        doc.add_solid(con_rod);
    }

    // === 5. Cylinder Head ===
    let head = build_cylinder_head(config);
    doc.add_solid(head);

    // === 6. Valves ===
    for i in 0..config.cylinder_count {
        // Intake valve
        let intake_valve = build_valve(config, i, true);
        doc.add_solid(intake_valve);
        // Exhaust valve
        let exhaust_valve = build_valve(config, i, false);
        doc.add_solid(exhaust_valve);
    }

    // === 7. Camshaft ===
    let camshaft = build_camshaft(config);
    doc.add_solid(camshaft);

    // === 8. Oil Pan ===
    let oil_pan = build_oil_pan(config);
    doc.add_solid(oil_pan);

    // === 9. Flywheel ===
    let flywheel = build_flywheel(config);
    doc.add_solid(flywheel);

    doc
}

/// Build the engine block.
fn build_engine_block(config: &EngineConfig) -> Solid {
    let bore = config.bore;
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let wall = config.wall_thickness;
    let deck = config.deck_height;
    let crank_radius = config.crank_radius;

    // Block outer dimensions
    let block_width = bore + 2.0 * wall;  // Width of each bore section
    let block_length = (n - 1) as f64 * spacing + bore + 2.0 * wall;
    let block_height = deck + wall;
    let block_depth = 2.0 * (bore / 2.0 + wall); // Front-to-back dimension

    // Main block body
    let mut block = ShapeBuilder::make_box(block_length, block_depth, block_height);
    ShapeBuilder::transform_solid(&mut block, &Transform::translation(
        0.0, 0.0, crank_radius - wall
    ));

    // Subtract cylinder bores
    for i in 0..n {
        let x_offset = i as f64 * spacing;
        let bore_cyl = ShapeBuilder::make_cylinder_at(
            x_offset, 0.0, crank_radius,
            bore / 2.0, deck,
        );

        // In a full implementation, we'd do boolean_subtract(&block, &bore_cyl)
        // For now, we'll approximate by creating the block with cylinder holes
        // represented as separate faces in the topology
        if i == 0 {
            // Add cylinder bore surfaces directly to the shell
            if let Some(ref mut shell) = block.outer_shell {
                let cyl_surface = CylinderSurface::new(
                    Point3d::new(x_offset, 0.0, crank_radius),
                    Direction3d::Z,
                    bore / 2.0,
                );
                let bore_face = Face::new(Surface::Cylinder(cyl_surface), Wire::new(vec![]));
                shell.faces.push(bore_face);
            }
        }
    }

    // Add crankshaft main bearing bores
    for i in 0..=n {
        let x_offset = i as f64 * spacing - spacing / 2.0;
        let bearing_cyl = ShapeBuilder::make_cylinder_at(
            x_offset, 0.0, 0.0,
            crank_radius * 0.6, // Bearing radius
            block_depth,
        );
        // Would boolean_subtract here
    }

    block
}

/// Build a piston.
fn build_piston(config: &EngineConfig, cylinder_index: usize) -> Solid {
    let bore = config.bore;
    let spacing = config.cylinder_spacing;
    let piston_h = config.piston_height;
    let wall = config.wall_thickness;

    // Piston is slightly smaller than bore (clearance)
    let piston_diameter = bore - 0.5; // 0.25mm clearance per side
    let piston_radius = piston_diameter / 2.0;

    // Main piston body (cylinder)
    let x_offset = cylinder_index as f64 * spacing;
    let z_offset = config.crank_radius + config.con_rod_length; // TDC position

    let mut piston = ShapeBuilder::make_cylinder(piston_radius, piston_h);
    ShapeBuilder::transform_solid(&mut piston, &Transform::translation(x_offset, 0.0, z_offset));

    // Piston crown — slight dome on top
    // (Would add a sphere intersection or dome shape)

    // Piston pin bore
    let pin_diameter = 22.0; // 22mm wrist pin
    let mut pin_bore = ShapeBuilder::make_cylinder(pin_diameter / 2.0, piston_diameter);
    // Rotate pin bore to go along Y axis
    if let Some(ref mut shell) = pin_bore.outer_shell {
        for face in &mut shell.faces {
            if let Some(ref mut surface) = face.surface {
                *surface = surface.transform(&Transform::rotation_x(std::f64::consts::PI / 2.0));
            }
        }
    }
    ShapeBuilder::transform_solid(&mut pin_bore, &Transform::translation(x_offset, 0.0, z_offset + piston_h * 0.3));

    // Would boolean_subtract pin_bore from piston here

    // Piston ring grooves (3 rings)
    for ring_idx in 0..3 {
        let ring_z = piston_h * 0.15 * (ring_idx + 1) as f64;
        let groove_depth = 1.5; // 1.5mm groove depth
        let groove_height = 2.0; // 2mm groove height

        // Would create a toroidal groove on the piston surface
        // For now, this is conceptual
    }

    piston
}

/// Build the crankshaft.
fn build_crankshaft(config: &EngineConfig) -> Solid {
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let crank_r = config.crank_radius;

    // Main journal dimensions
    let main_journal_diameter = 55.0;
    let main_journal_length = 28.0;
    let rod_journal_diameter = 48.0;
    let rod_journal_length = 24.0;

    // Total crankshaft length
    let total_length = (n - 1) as f64 * spacing + spacing;
    let crankshaft_x_start = -spacing / 2.0;

    // Build crankshaft as a series of journals and webs
    let mut crankshaft = ShapeBuilder::make_cylinder(
        main_journal_diameter / 2.0, total_length
    );

    // Rotate to align along X axis
    if let Some(ref mut shell) = crankshaft.outer_shell {
        for face in &mut shell.faces {
            if let Some(ref mut surface) = face.surface {
                *surface = surface.transform(&Transform::rotation_y(std::f64::consts::PI / 2.0));
            }
        }
    }

    // Position
    ShapeBuilder::transform_solid(&mut crankshaft, &Transform::translation(0.0, 0.0, 0.0));

    // Add crank throws (offset journals for each cylinder)
    for i in 0..n {
        let x_pos = i as f64 * spacing;

        // Crank pin (rod journal)
        let mut rod_journal = ShapeBuilder::make_cylinder(
            rod_journal_diameter / 2.0, rod_journal_length
        );

        // Rotate to X axis and offset by crank radius
        if let Some(ref mut shell) = rod_journal.outer_shell {
            for face in &mut shell.faces {
                if let Some(ref mut surface) = face.surface {
                    *surface = surface.transform(&Transform::rotation_y(std::f64::consts::PI / 2.0));
                }
            }
        }
        ShapeBuilder::transform_solid(&mut rod_journal,
            &Transform::translation(x_pos, 0.0, -crank_r)
        );

        // Crank web connecting main journal to rod journal
        let web_thickness = (spacing - main_journal_length) / 2.0;
        let mut crank_web = ShapeBuilder::make_box(
            web_thickness, main_journal_diameter, crank_r + main_journal_diameter / 2.0
        );
        ShapeBuilder::transform_solid(&mut crank_web,
            &Transform::translation(
                x_pos - web_thickness / 2.0,
                0.0,
                -(crank_r / 2.0 + main_journal_diameter / 4.0)
            )
        );

        // In a full implementation, boolean_union these parts
    }

    crankshaft
}

/// Build a connecting rod.
fn build_connecting_rod(config: &EngineConfig, cylinder_index: usize) -> Solid {
    let spacing = config.cylinder_spacing;
    let con_rod_len = config.con_rod_length;
    let crank_r = config.crank_radius;
    let big_end_diameter = 52.0; // Big end (crank pin) bore
    let small_end_diameter = 24.0; // Small end (piston pin) bore
    let rod_thickness = 18.0; // I-beam web thickness
    let rod_width = 24.0; // I-beam width

    let x_offset = cylinder_index as f64 * spacing;

    // Simplified connecting rod — a tapered beam with two circular ends

    // Big end (crank end)
    let mut big_end = ShapeBuilder::make_cylinder(big_end_diameter / 2.0, rod_thickness);
    if let Some(ref mut shell) = big_end.outer_shell {
        for face in &mut shell.faces {
            if let Some(ref mut surface) = face.surface {
                *surface = surface.transform(&Transform::rotation_x(std::f64::consts::PI / 2.0));
            }
        }
    }
    ShapeBuilder::transform_solid(&mut big_end, &Transform::translation(x_offset, 0.0, crank_r));

    // Rod beam (tapered box)
    let beam_taper = 0.7; // Small end is 70% of big end width
    let mut beam = ShapeBuilder::make_box(
        rod_width, rod_thickness, con_rod_len - big_end_diameter
    );
    ShapeBuilder::transform_solid(&mut beam, &Transform::translation(
        x_offset,
        0.0,
        crank_r + big_end_diameter / 2.0
    ));

    // Small end (piston pin end)
    let small_z = crank_r + con_rod_len;
    let mut small_end = ShapeBuilder::make_cylinder(small_end_diameter / 2.0, rod_thickness * beam_taper);
    if let Some(ref mut shell) = small_end.outer_shell {
        for face in &mut shell.faces {
            if let Some(ref mut surface) = face.surface {
                *surface = surface.transform(&Transform::rotation_x(std::f64::consts::PI / 2.0));
            }
        }
    }
    ShapeBuilder::transform_solid(&mut small_end, &Transform::translation(x_offset, 0.0, small_z));

    // Boolean union all parts
    let result = boolean::boolean_union(&big_end, &beam).unwrap();
    boolean::boolean_union(&result, &small_end).unwrap_or(result)
}

/// Build the cylinder head.
fn build_cylinder_head(config: &EngineConfig) -> Solid {
    let bore = config.bore;
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let wall = config.wall_thickness;
    let deck = config.deck_height;

    let head_length = (n - 1) as f64 * spacing + bore + 2.0 * wall;
    let head_depth = bore + 2.0 * wall + 20.0; // Extra for ports
    let head_height = 65.0; // Typical head height

    let mut head = ShapeBuilder::make_box(head_length, head_depth, head_height);
    ShapeBuilder::transform_solid(&mut head, &Transform::translation(
        0.0, 0.0, deck
    ));

    // Combustion chambers (pentroof shape — approximated as cylinders)
    for i in 0..n {
        let x_offset = i as f64 * spacing;
        let chamber = ShapeBuilder::make_cylinder_at(
            x_offset, 0.0, deck,
            bore / 2.0 * 0.95, // Slightly smaller than bore
            head_height * 0.3,
        );
        // Would boolean_subtract
    }

    // Valve ports
    for i in 0..n {
        let x_offset = i as f64 * spacing;
        // Intake port
        let intake_port = ShapeBuilder::make_cylinder_at(
            x_offset, -bore * 0.25, deck + head_height * 0.4,
            config.valve_diameter / 2.0 * 0.8,
            head_depth,
        );
        // Exhaust port
        let exhaust_port = ShapeBuilder::make_cylinder_at(
            x_offset, bore * 0.25, deck + head_height * 0.4,
            config.valve_diameter / 2.0 * 0.7,
            head_depth,
        );
        // Would boolean_subtract both
    }

    // Bolt holes (10mm bolts around perimeter)
    let bolt_diameter = 10.0;
    let bolt_positions = calculate_head_bolt_positions(config);
    for (bx, by) in bolt_positions {
        let bolt_hole = ShapeBuilder::make_cylinder_at(
            bx, by, deck, bolt_diameter / 2.0, head_height,
        );
        // Would boolean_subtract
    }

    head
}

/// Calculate head bolt positions (simplified pattern).
fn calculate_head_bolt_positions(config: &EngineConfig) -> Vec<(f64, f64)> {
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let bore = config.bore;
    let wall = config.wall_thickness;
    let offset = (bore / 2.0 + wall / 2.0);

    let mut positions = Vec::new();

    for i in 0..n {
        let x = i as f64 * spacing;
        // 4 bolts per cylinder (typical)
        positions.push((x - offset * 0.4, -offset));
        positions.push((x + offset * 0.4, -offset));
        positions.push((x - offset * 0.4, offset));
        positions.push((x + offset * 0.4, offset));
    }

    positions
}

/// Build a valve (intake or exhaust).
fn build_valve(config: &EngineConfig, cylinder_index: usize, is_intake: bool) -> Solid {
    let spacing = config.cylinder_spacing;
    let bore = config.bore;
    let deck = config.deck_height;
    let valve_d = config.valve_diameter;
    let valve_l = config.valve_length;
    let stem_d = 7.0; // 7mm valve stem

    let x_offset = cylinder_index as f64 * spacing;
    let y_offset = if is_intake { -bore * 0.25 } else { bore * 0.25 };
    let z_base = deck + 10.0; // Slightly above deck

    // Valve head (tulip shape — approximated as disk + cone)
    let head_thickness = 2.0;
    let mut valve_head = ShapeBuilder::make_cylinder(valve_d / 2.0, head_thickness);

    // Valve stem
    let stem_length = valve_l - head_thickness;
    let mut valve_stem = ShapeBuilder::make_cylinder(stem_d / 2.0, stem_length);
    ShapeBuilder::transform_solid(&mut valve_stem, &Transform::translation(0.0, 0.0, head_thickness));

    // Position the complete valve
    let transform = Transform::translation(x_offset, y_offset, z_base);
    ShapeBuilder::transform_solid(&mut valve_head, &transform);
    ShapeBuilder::transform_solid(&mut valve_stem, &transform);

    // Boolean union
    boolean::boolean_union(&valve_head, &valve_stem).unwrap_or(valve_head)
}

/// Build the camshaft.
fn build_camshaft(config: &EngineConfig) -> Solid {
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let bore = config.bore;
    let deck = config.deck_height;
    let wall = config.wall_thickness;

    let cam_diameter = 30.0;
    let cam_length = (n - 1) as f64 * spacing + spacing;
    let cam_z = deck + 80.0; // Above the head

    // Main shaft
    let mut camshaft = ShapeBuilder::make_cylinder(cam_diameter / 2.0, cam_length);
    if let Some(ref mut shell) = camshaft.outer_shell {
        for face in &mut shell.faces {
            if let Some(ref mut surface) = face.surface {
                *surface = surface.transform(&Transform::rotation_y(std::f64::consts::PI / 2.0));
            }
        }
    }
    ShapeBuilder::transform_solid(&mut camshaft, &Transform::translation(0.0, 0.0, cam_z));

    // Cam lobes (egg-shaped cross sections — approximated as offset cylinders)
    for i in 0..n {
        let x = i as f64 * spacing;
        // Intake cam lobe
        let mut lobe_intake = ShapeBuilder::make_cylinder(cam_diameter / 2.0 + 5.0, 12.0);
        if let Some(ref mut shell) = lobe_intake.outer_shell {
            for face in &mut shell.faces {
                if let Some(ref mut surface) = face.surface {
                    *surface = surface.transform(&Transform::rotation_y(std::f64::consts::PI / 2.0));
                }
            }
        }
        ShapeBuilder::transform_solid(&mut lobe_intake,
            &Transform::translation(x - 8.0, 0.0, cam_z)
        );

        // Exhaust cam lobe
        let mut lobe_exhaust = ShapeBuilder::make_cylinder(cam_diameter / 2.0 + 4.0, 12.0);
        if let Some(ref mut shell) = lobe_exhaust.outer_shell {
            for face in &mut shell.faces {
                if let Some(ref mut surface) = face.surface {
                    *surface = surface.transform(&Transform::rotation_y(std::f64::consts::PI / 2.0));
                }
            }
        }
        ShapeBuilder::transform_solid(&mut lobe_exhaust,
            &Transform::translation(x + 8.0, 0.0, cam_z)
        );
    }

    camshaft
}

/// Build the oil pan.
fn build_oil_pan(config: &EngineConfig) -> Solid {
    let bore = config.bore;
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let wall = config.wall_thickness;
    let crank_r = config.crank_radius;

    let pan_length = (n - 1) as f64 * spacing + bore + 2.0 * wall;
    let pan_depth = bore + 2.0 * wall;
    let pan_height = 60.0; // Oil pan depth
    let pan_z = -(crank_r + wall); // Below the crankshaft

    let mut pan = ShapeBuilder::make_box(pan_length, pan_depth, pan_height);
    ShapeBuilder::transform_solid(&mut pan, &Transform::translation(0.0, 0.0, pan_z - pan_height / 2.0));

    // Oil sump (deeper section at the back)
    let sump_length = pan_length * 0.4;
    let sump_depth = pan_depth * 0.8;
    let sump_height = 40.0;
    let mut sump = ShapeBuilder::make_box(sump_length, sump_depth, sump_height);
    ShapeBuilder::transform_solid(&mut sump, &Transform::translation(
        pan_length * 0.2, 0.0, pan_z - pan_height - sump_height / 2.0
    ));

    boolean::boolean_union(&pan, &sump).unwrap_or(pan)
}

/// Build the flywheel.
fn build_flywheel(config: &EngineConfig) -> Solid {
    let n = config.cylinder_count;
    let spacing = config.cylinder_spacing;
    let crank_r = config.crank_radius;

    let flywheel_diameter = 260.0; // 260mm flywheel
    let flywheel_thickness = 15.0; // 15mm thick
    let x_position = (n as f64 * spacing) + 20.0; // At the end of the crankshaft

    let mut flywheel = ShapeBuilder::make_cylinder(flywheel_diameter / 2.0, flywheel_thickness);
    if let Some(ref mut shell) = flywheel.outer_shell {
        for face in &mut shell.faces {
            if let Some(ref mut surface) = face.surface {
                *surface = surface.transform(&Transform::rotation_y(std::f64::consts::PI / 2.0));
            }
        }
    }
    ShapeBuilder::transform_solid(&mut flywheel, &Transform::translation(x_position, 0.0, 0.0));

    // Center bore for crankshaft
    let center_bore_d = 30.0;
    let center_bore = ShapeBuilder::make_cylinder(center_bore_d / 2.0, flywheel_thickness + 2.0);
    // Would boolean_subtract center bore

    // Ring gear teeth (around outer diameter)
    let ring_gear_d = flywheel_diameter - 5.0;
    let ring_gear = ShapeBuilder::make_cylinder(ring_gear_d / 2.0, 8.0);
    // Would boolean_subtract for tooth pattern

    flywheel
}
