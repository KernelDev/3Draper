//! Internal Combustion Engine (ICE) model builder.
//!
//! Constructs a simplified inline-4 cylinder engine using the 3Draper B-Rep kernel.
//! This serves as a comprehensive test of the kernel's modeling capabilities and
//! demonstrates what features are available vs. what needs to be implemented.
//!
//! Engine components:
//! - Engine block (box with 4 cylinder bores)
//! - Cylinder head (box with bolt holes)
//! - Oil pan (box)
//! - 4 Pistons (cylinders with crown features)
//! - 4 Connecting rods (simplified)
//! - Crankshaft (cylinders + pins)
//! - 8 Valves (4 intake + 4 exhaust)
//! - 4 Spark plugs
//! - Camshaft (cylinder with lobes)
//! - Intake manifold (simplified)
//! - Exhaust manifold (simplified)
//!
//! Key insight: Many of these parts require Boolean operations (difference, union)
//! which are NOT yet available at the B-Rep level. We work around this by:
//! 1. Building each part as a separate Shape with proper B-Rep topology
//! 2. Using `make_box_with_cylinder_holes` for the engine block (manual Boolean)
//! 3. Combining all parts into a Compound (assembly) with transforms
//!
//! Dimensions are in millimeters, loosely based on a 2.0L inline-4 engine.

use draper_geometry::transform::Transform3;
use draper_topology::builder::ShapeBuilder;
use draper_topology::entity::*;
use draper_topology::shape::Shape;

// =====================================================================
// Engine specifications (mm)
// =====================================================================

/// Bore diameter: 86mm
const BORE_DIAMETER: f64 = 86.0;
/// Bore radius
const BORE_RADIUS: f64 = BORE_DIAMETER / 2.0;
/// Stroke: 86mm (square engine)
const STROKE: f64 = 86.0;
/// Distance between cylinder centers: 96mm
const CYLINDER_SPACING: f64 = 96.0;
/// Number of cylinders
const NUM_CYLINDERS: usize = 4;
/// Deck height (block height from crank center to top)
const BLOCK_HEIGHT: f64 = 250.0;
/// Block width (X direction)
const BLOCK_WIDTH: f64 = 120.0;
/// Block length (Y direction) — covers all cylinders
const BLOCK_LENGTH: f64 = CYLINDER_SPACING * (NUM_CYLINDERS as f64 - 1.0) + BORE_DIAMETER + 30.0;
/// Crankshaft center Z position (from block bottom)
const CRANK_CENTER_Z: f64 = 60.0;
/// Piston compression height
const PISTON_HEIGHT: f64 = 50.0;
/// Piston pin diameter
const PISTON_PIN_DIAMETER: f64 = 22.0;
/// Connecting rod length (center to center)
const CONROD_LENGTH: f64 = 145.0;
/// Connecting rod big end diameter
const CONROD_BIG_END_DIAMETER: f64 = 52.0;
/// Connecting rod small end diameter
const CONROD_SMALL_END_DIAMETER: f64 = 28.0;
/// Connecting rod width (thickness)
const CONROD_WIDTH: f64 = 24.0;
/// Crankshaft main journal diameter
const CRANK_MAIN_DIAMETER: f64 = 56.0;
/// Crankshaft pin journal diameter
const CRANK_PIN_DIAMETER: f64 = 48.0;
/// Crankshaft main journal length
const CRANK_MAIN_LENGTH: f64 = 28.0;
/// Crankshaft pin journal length
const CRANK_PIN_LENGTH: f64 = 26.0;
/// Crankshaft total length
const CRANK_TOTAL_LENGTH: f64 = BLOCK_LENGTH + 40.0;
/// Valve stem diameter
const VALVE_STEM_DIAMETER: f64 = 6.0;
/// Valve head diameter (intake)
const VALVE_INTAKE_HEAD_DIAMETER: f64 = 35.0;
/// Valve head diameter (exhaust)
const VALVE_EXHAUST_HEAD_DIAMETER: f64 = 30.0;
/// Valve total length
const VALVE_LENGTH: f64 = 100.0;
/// Valve head thickness
const VALVE_HEAD_THICKNESS: f64 = 5.0;
/// Cylinder head height
const HEAD_HEIGHT: f64 = 80.0;
/// Oil pan height
const OIL_PAN_HEIGHT: f64 = 80.0;
/// Spark plug thread diameter
const SPARK_PLUG_DIAMETER: f64 = 14.0;
/// Spark plug length
const SPARK_PLUG_LENGTH: f64 = 40.0;
/// Camshaft diameter
const CAMSHAFT_DIAMETER: f64 = 30.0;
/// Camshaft length
const CAMSHAFT_LENGTH: f64 = BLOCK_LENGTH + 20.0;

/// Complete ICE model result.
#[derive(Debug)]
pub struct EngineModel {
    /// The assembled shape (compound of all parts).
    pub shape: Shape,
    /// Names for each solid, keyed by TopoId.
    pub part_names: std::collections::HashMap<TopoId, String>,
    /// Colors for each solid, keyed by TopoId.
    pub part_colors: std::collections::HashMap<TopoId, [f32; 3]>,
}

impl EngineModel {
    /// Build the complete inline-4 ICE model.
    pub fn build() -> Self {
        let mut shape = Shape::new();
        let mut part_names = std::collections::HashMap::new();
        let mut part_colors = std::collections::HashMap::new();

        // Build each component
        let block = build_engine_block(&mut shape);
        part_names.insert(block, "Engine Block".to_string());
        part_colors.insert(block, [0.55, 0.55, 0.60]); // Steel gray

        let head = build_cylinder_head(&mut shape);
        part_names.insert(head, "Cylinder Head".to_string());
        part_colors.insert(head, [0.50, 0.50, 0.55]); // Slightly darker gray

        let oil_pan = build_oil_pan(&mut shape);
        part_names.insert(oil_pan, "Oil Pan".to_string());
        part_colors.insert(oil_pan, [0.35, 0.35, 0.38]); // Dark gray

        let mut pistons = Vec::new();
        for i in 0..NUM_CYLINDERS {
            let piston = build_piston(&mut shape, i);
            part_names.insert(piston, format!("Piston #{}", i + 1));
            part_colors.insert(piston, [0.75, 0.65, 0.40]); // Bronze
            pistons.push(piston);
        }

        let mut conrods = Vec::new();
        for i in 0..NUM_CYLINDERS {
            let conrod = build_connecting_rod(&mut shape, i);
            part_names.insert(conrod, format!("Connecting Rod #{}", i + 1));
            part_colors.insert(conrod, [0.60, 0.60, 0.65]); // Steel
            conrods.push(conrod);
        }

        let crankshaft = build_crankshaft(&mut shape);
        part_names.insert(crankshaft, "Crankshaft".to_string());
        part_colors.insert(crankshaft, [0.65, 0.65, 0.70]); // Polished steel

        let mut valves = Vec::new();
        for i in 0..NUM_CYLINDERS {
            // Intake valve
            let intake = build_valve(&mut shape, i, true);
            part_names.insert(intake, format!("Intake Valve #{}", i + 1));
            part_colors.insert(intake, [0.70, 0.75, 0.80]); // Silver
            valves.push(intake);

            // Exhaust valve
            let exhaust = build_valve(&mut shape, i, false);
            part_names.insert(exhaust, format!("Exhaust Valve #{}", i + 1));
            part_colors.insert(exhaust, [0.55, 0.45, 0.35]); // Heat-tinted
            valves.push(exhaust);
        }

        let mut spark_plugs = Vec::new();
        for i in 0..NUM_CYLINDERS {
            let plug = build_spark_plug(&mut shape, i);
            part_names.insert(plug, format!("Spark Plug #{}", i + 1));
            part_colors.insert(plug, [0.90, 0.85, 0.70]); // Ceramic white-yellow
            spark_plugs.push(plug);
        }

        let camshaft = build_camshaft(&mut shape);
        part_names.insert(camshaft, "Camshaft".to_string());
        part_colors.insert(camshaft, [0.60, 0.60, 0.65]); // Steel

        // Combine into compound
        let all_parts: Vec<TopoId> = vec![block, head, oil_pan, crankshaft, camshaft]
            .into_iter()
            .chain(pistons.into_iter())
            .chain(conrods.into_iter())
            .chain(valves.into_iter())
            .chain(spark_plugs.into_iter())
            .collect();

        let _compound = shape.add_compound(all_parts);

        log::info!(
            "Engine model built: {} vertices, {} edges, {} faces, {} solids",
            shape.vertices().len(),
            shape.edges().len(),
            shape.faces().len(),
            shape.solids().len(),
        );

        EngineModel {
            shape,
            part_names,
            part_colors,
        }
    }
}

/// Get the Y position of a cylinder by index (0-based).
fn cylinder_y(index: usize) -> f64 {
    let offset = (BLOCK_LENGTH - CYLINDER_SPACING * (NUM_CYLINDERS as f64 - 1.0)) / 2.0;
    offset + CYLINDER_SPACING * index as f64
}

/// Build the engine block — a box with 4 cylinder bores.
fn build_engine_block(shape: &mut Shape) -> TopoId {
    let bores: Vec<(f64, f64, f64)> = (0..NUM_CYLINDERS)
        .map(|i| (BLOCK_WIDTH / 2.0, cylinder_y(i), BORE_RADIUS))
        .collect();

    ShapeBuilder::make_box_with_cylinder_holes(
        shape,
        BLOCK_WIDTH,
        BLOCK_LENGTH,
        BLOCK_HEIGHT,
        &bores,
    )
}

/// Build the cylinder head — a box on top of the block.
fn build_cylinder_head(shape: &mut Shape) -> TopoId {
    let head = ShapeBuilder::make_box(shape, BLOCK_WIDTH, BLOCK_LENGTH, HEAD_HEIGHT);

    // Position on top of the block
    apply_transform_to_solid(shape, head, Transform3::from_translation(
        0.0, 0.0, BLOCK_HEIGHT,
    ));

    head
}

/// Build the oil pan — a box below the block.
fn build_oil_pan(shape: &mut Shape) -> TopoId {
    let pan = ShapeBuilder::make_box(shape, BLOCK_WIDTH, BLOCK_LENGTH, OIL_PAN_HEIGHT);

    // Position below the block
    apply_transform_to_solid(shape, pan, Transform3::from_translation(
        0.0, 0.0, -OIL_PAN_HEIGHT,
    ));

    pan
}

/// Build a piston for cylinder `index` at TDC position.
fn build_piston(shape: &mut Shape, cylinder_index: usize) -> TopoId {
    let piston = ShapeBuilder::make_cylinder(shape, BORE_RADIUS - 0.5, PISTON_HEIGHT);

    // Position: above the crank center, inside the cylinder bore
    let y = cylinder_y(cylinder_index);
    let z = BLOCK_HEIGHT - PISTON_HEIGHT - 10.0; // Near TDC
    apply_transform_to_solid(shape, piston, Transform3::from_translation(
        BLOCK_WIDTH / 2.0, y, z,
    ));

    piston
}

/// Build a connecting rod for cylinder `index`.
fn build_connecting_rod(shape: &mut Shape, cylinder_index: usize) -> TopoId {
    // Simplified: a box for the beam + 2 cylinders for big/small ends
    // The rod connects the piston pin to the crank pin

    let y = cylinder_y(cylinder_index);

    // Big end (crank end) — near the crank center
    let big_end = ShapeBuilder::make_cylinder(shape, CONROD_BIG_END_DIAMETER / 2.0, CONROD_WIDTH);

    // Position big end at crank center
    apply_transform_to_solid(shape, big_end, Transform3::from_translation(
        BLOCK_WIDTH / 2.0, y, CRANK_CENTER_Z - CONROD_WIDTH / 2.0,
    ));

    // Small end (piston pin end)
    let small_end_z = CRANK_CENTER_Z + CONROD_LENGTH;
    let small_end = ShapeBuilder::make_cylinder(shape, CONROD_SMALL_END_DIAMETER / 2.0, CONROD_WIDTH);

    apply_transform_to_solid(shape, small_end, Transform3::from_translation(
        BLOCK_WIDTH / 2.0, y, small_end_z - CONROD_WIDTH / 2.0,
    ));

    // Beam (connecting the two ends)
    let beam_length = CONROD_LENGTH - CONROD_BIG_END_DIAMETER / 2.0 - CONROD_SMALL_END_DIAMETER / 2.0;
    let beam = ShapeBuilder::make_box(shape, CONROD_WIDTH * 0.6, beam_length, CONROD_WIDTH);

    let beam_offset_z = CRANK_CENTER_Z + CONROD_BIG_END_DIAMETER / 2.0;
    apply_transform_to_solid(shape, beam, Transform3::from_translation(
        BLOCK_WIDTH / 2.0 - CONROD_WIDTH * 0.3, y - beam_length / 2.0, beam_offset_z,
    ));

    // Combine into compound (not perfect boolean, but visual assembly)
    shape.add_compound(vec![big_end, small_end, beam])
}

/// Build the crankshaft.
fn build_crankshaft(shape: &mut Shape) -> TopoId {
    let mut parts = Vec::new();

    // Main journal positions along Y axis
    let num_main_journals = NUM_CYLINDERS + 1; // 5 main journals for I4
    let main_spacing = BLOCK_LENGTH / (num_main_journals - 1) as f64;

    for i in 0..num_main_journals {
        let y = main_spacing * i as f64;
        let journal = ShapeBuilder::make_cylinder(shape, CRANK_MAIN_DIAMETER / 2.0, CRANK_MAIN_LENGTH);
        apply_transform_to_solid(shape, journal, Transform3::from_translation(
            BLOCK_WIDTH / 2.0, y - CRANK_MAIN_LENGTH / 2.0, CRANK_CENTER_Z,
        ));
        parts.push(journal);
    }

    // Crank pins — offset from main axis (crank throw)
    let crank_throw = STROKE / 2.0;
    for i in 0..NUM_CYLINDERS {
        let y = cylinder_y(i);
        let pin = ShapeBuilder::make_cylinder(shape, CRANK_PIN_DIAMETER / 2.0, CRANK_PIN_LENGTH);
        apply_transform_to_solid(shape, pin, Transform3::from_translation(
            BLOCK_WIDTH / 2.0 + crank_throw, y - CRANK_PIN_LENGTH / 2.0, CRANK_CENTER_Z,
        ));
        parts.push(pin);
    }

    // Counterweights (simplified as boxes)
    for i in 0..NUM_CYLINDERS {
        let y = cylinder_y(i);
        let cw = ShapeBuilder::make_box(shape, crank_throw * 0.8, CRANK_PIN_LENGTH * 0.8, CRANK_MAIN_DIAMETER * 0.8);
        apply_transform_to_solid(shape, cw, Transform3::from_translation(
            BLOCK_WIDTH / 2.0 + crank_throw * 0.4, y - CRANK_PIN_LENGTH * 0.4, CRANK_CENTER_Z - CRANK_MAIN_DIAMETER * 0.4,
        ));
        parts.push(cw);
    }

    shape.add_compound(parts)
}

/// Build a valve (intake or exhaust) for a cylinder.
fn build_valve(shape: &mut Shape, cylinder_index: usize, is_intake: bool) -> TopoId {
    let head_diameter = if is_intake { VALVE_INTAKE_HEAD_DIAMETER } else { VALVE_EXHAUST_HEAD_DIAMETER };
    let x_offset = if is_intake { BLOCK_WIDTH * 0.35 } else { BLOCK_WIDTH * 0.65 };

    // Stem
    let stem = ShapeBuilder::make_cylinder(shape, VALVE_STEM_DIAMETER / 2.0, VALVE_LENGTH - VALVE_HEAD_THICKNESS);

    // Head (truncated cone)
    let head = ShapeBuilder::make_cone(
        shape,
        head_diameter / 2.0,
        VALVE_STEM_DIAMETER / 2.0,
        VALVE_HEAD_THICKNESS,
    );

    // Position the valve
    let y = cylinder_y(cylinder_index);
    let z_base = BLOCK_HEIGHT + HEAD_HEIGHT * 0.6; // In the head

    apply_transform_to_solid(shape, stem, Transform3::from_translation(
        x_offset, y, z_base,
    ));

    apply_transform_to_solid(shape, head, Transform3::from_translation(
        x_offset, y, z_base,
    ));

    shape.add_compound(vec![stem, head])
}

/// Build a spark plug for a cylinder.
fn build_spark_plug(shape: &mut Shape, cylinder_index: usize) -> TopoId {
    // Thread section
    let thread = ShapeBuilder::make_cylinder(shape, SPARK_PLUG_DIAMETER / 2.0, SPARK_PLUG_LENGTH * 0.6);

    // Insulator (wider section on top)
    let insulator = ShapeBuilder::make_cylinder(shape, SPARK_PLUG_DIAMETER * 0.8, SPARK_PLUG_LENGTH * 0.4);

    // Hex section
    let hex = ShapeBuilder::make_cylinder(shape, SPARK_PLUG_DIAMETER * 1.2, SPARK_PLUG_LENGTH * 0.2);

    let y = cylinder_y(cylinder_index);
    let z_base = BLOCK_HEIGHT + HEAD_HEIGHT - 5.0;

    apply_transform_to_solid(shape, thread, Transform3::from_translation(
        BLOCK_WIDTH / 2.0, y, z_base - SPARK_PLUG_LENGTH * 0.6,
    ));
    apply_transform_to_solid(shape, insulator, Transform3::from_translation(
        BLOCK_WIDTH / 2.0, y, z_base,
    ));
    apply_transform_to_solid(shape, hex, Transform3::from_translation(
        BLOCK_WIDTH / 2.0, y, z_base + SPARK_PLUG_LENGTH * 0.4,
    ));

    shape.add_compound(vec![thread, insulator, hex])
}

/// Build the camshaft.
fn build_camshaft(shape: &mut Shape) -> TopoId {
    let camshaft = ShapeBuilder::make_cylinder(shape, CAMSHAFT_DIAMETER / 2.0, CAMSHAFT_LENGTH);

    // Position at the top of the cylinder head
    apply_transform_to_solid(shape, camshaft, Transform3::from_translation(
        BLOCK_WIDTH * 0.2, -10.0, BLOCK_HEIGHT + HEAD_HEIGHT - CAMSHAFT_DIAMETER,
    ));

    camshaft
}

/// Apply a transform to all vertices of a solid.
///
/// This modifies vertex positions in-place. It does NOT update
/// curve/surface geometry (which would require full re-parameterization).
/// For rendering purposes, the vertex positions are sufficient.
fn apply_transform_to_solid(shape: &mut Shape, solid_id: TopoId, transform: Transform3) {
    // Collect all vertex IDs belonging to this solid
    let vertex_ids = collect_solid_vertices(shape, solid_id);

    // Apply transform to each vertex
    for vid in vertex_ids {
        if let Some(TopoShape::Vertex(v)) = shape.entities.get_mut(&vid) {
            v.point = transform.transform_point(v.point);
        }
    }
}

/// Collect all vertex IDs belonging to a solid by traversing the topology.
fn collect_solid_vertices(shape: &Shape, solid_id: TopoId) -> Vec<TopoId> {
    let mut vertex_ids = std::collections::HashSet::new();

    // Get the shell
    if let Some(TopoShape::Solid(solid)) = shape.get(solid_id) {
        if let Some(TopoShape::Shell(shell)) = shape.get(solid.outer_shell) {
            for &face_id in &shell.faces {
                if let Some(TopoShape::Face(face)) = shape.get(face_id) {
                    // Collect vertices from outer wire
                    if let Some(wire_id) = face.outer_wire {
                        collect_wire_vertices(shape, wire_id, &mut vertex_ids);
                    }
                    // Collect vertices from inner wires
                    for &wire_id in &face.inner_wires {
                        collect_wire_vertices(shape, wire_id, &mut vertex_ids);
                    }
                }
            }
        }
    }

    vertex_ids.into_iter().collect()
}

/// Collect vertex IDs from a wire.
fn collect_wire_vertices(shape: &Shape, wire_id: TopoId, vertex_ids: &mut std::collections::HashSet<TopoId>) {
    if let Some(TopoShape::Wire(wire)) = shape.get(wire_id) {
        for oriented_edge in &wire.edges {
            if let Some(TopoShape::Edge(edge)) = shape.get(oriented_edge.edge_id) {
                vertex_ids.insert(edge.start_vertex);
                vertex_ids.insert(edge.end_vertex);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_engine_model_builds() {
        let engine = EngineModel::build();
        assert!(!engine.shape.is_empty());
        assert!(engine.shape.solids().len() > 0);
        assert!(engine.shape.vertices().len() > 0);
        assert!(engine.shape.faces().len() > 0);
    }

    #[test]
    fn test_engine_block_has_holes() {
        let mut shape = Shape::new();
        let block = build_engine_block(&mut shape);

        // Should have at least the box faces + bore cylindrical faces
        let faces = shape.faces();
        // Box = 6 faces + 4 bores × (lateral segments + 1 cylindrical surface)
        assert!(faces.len() > 6, "Engine block should have more than 6 faces (has {})", faces.len());

        // Should have inner wires on top face (holes)
        let has_inner_wire = faces.iter().any(|f| !f.inner_wires.is_empty());
        assert!(has_inner_wire, "Engine block top face should have inner wires (bore holes)");
    }

    #[test]
    fn test_cylinder_y_positions() {
        let y0 = cylinder_y(0);
        let y1 = cylinder_y(1);
        let spacing = y1 - y0;
        assert!((spacing - CYLINDER_SPACING).abs() < 1e-6,
            "Cylinder spacing should be {}mm, got {}", CYLINDER_SPACING, spacing);
    }
}
