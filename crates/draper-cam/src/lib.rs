// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! CAM toolpath generation — 2.5D milling, drilling, G-code export.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 3.2: provides CAM operations
//! for generating CNC toolpaths from 2D profiles:
//!
//! - **Tool library**: end mills, ball mills, drills with standard sizes.
//! - **Contour milling**: follow a 2D profile at a given depth.
//! - **Pocket milling**: clear a rectangular or circular pocket.
//! - **Drilling**: peck-drill cycles at specified locations.
//! - **G-code generation**: export toolpaths as G-code for CNC machines.
//!
//! # G-code Format
//!
//! Standard ISO G-code (RS-274):
//! - G00: rapid positioning
//! - G01: linear interpolation (feed rate)
//! - G02/G03: circular interpolation (CW/CCW)
//! - G81: drilling cycle
//! - M03/M05: spindle on/off
//! - M30: program end

use std::f64::consts::PI;
use std::fmt::Write;

// ============================================================
// Error types
// ============================================================

#[derive(Debug, Clone, thiserror::Error)]
pub enum CamError {
    #[error("Empty toolpath — no operations")]
    EmptyToolpath,

    #[error("Invalid tool diameter: {0} (must be > 0)")]
    InvalidToolDiameter(f64),

    #[error("Invalid feed rate: {0} (must be > 0)")]
    InvalidFeedRate(f64),

    #[error("Invalid depth: {0}")]
    InvalidDepth(f64),

    #[error("Invalid stepover: {0} (must be 0..=1)")]
    InvalidStepover(f64),
}

// ============================================================
// Tool Library
// ============================================================

/// CAM tool type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolType {
    /// Flat-end mill for profiling and pocketing.
    EndMill,
    /// Ball-nose mill for 3D sculpting.
    BallMill,
    /// Twist drill for holes.
    Drill,
    /// Face mill for surface facing.
    FaceMill,
}

impl ToolType {
    pub fn name(&self) -> &'static str {
        match self {
            ToolType::EndMill => "End Mill",
            ToolType::BallMill => "Ball Mill",
            ToolType::Drill => "Drill",
            ToolType::FaceMill => "Face Mill",
        }
    }
}

/// A CNC cutting tool.
#[derive(Debug, Clone)]
pub struct Tool {
    /// Tool number (T1, T2, ...).
    pub number: u32,
    /// Tool type.
    pub tool_type: ToolType,
    /// Tool diameter in mm.
    pub diameter: f64,
    /// Flute count.
    pub flutes: u32,
    /// Default feed rate in mm/min.
    pub feed_rate: f64,
    /// Default plunge rate in mm/min.
    pub plunge_rate: f64,
    /// Default spindle speed in RPM.
    pub spindle_rpm: f64,
}

impl Tool {
    /// Create a new tool.
    pub fn new(number: u32, tool_type: ToolType, diameter: f64) -> Result<Self, CamError> {
        if diameter <= 0.0 {
            return Err(CamError::InvalidToolDiameter(diameter));
        }
        Ok(Self {
            number,
            tool_type,
            diameter,
            flutes: 2,
            feed_rate: 500.0,
            plunge_rate: 100.0,
            spindle_rpm: 8000.0,
        })
    }

    /// Standard 6mm 2-flute end mill.
    pub fn endmill_6mm() -> Self {
        Self::new(1, ToolType::EndMill, 6.0).unwrap()
    }

    /// Standard 3mm 2-flute end mill.
    pub fn endmill_3mm() -> Self {
        Self::new(2, ToolType::EndMill, 3.0).unwrap()
    }

    /// Standard 10mm 4-flute face mill.
    pub fn facemill_10mm() -> Self {
        let mut t = Self::new(3, ToolType::FaceMill, 10.0).unwrap();
        t.flutes = 4;
        t.feed_rate = 800.0;
        t
    }

    /// Standard 5mm drill.
    pub fn drill_5mm() -> Self {
        let mut t = Self::new(4, ToolType::Drill, 5.0).unwrap();
        t.feed_rate = 150.0;
        t.plunge_rate = 50.0;
        t.spindle_rpm = 3000.0;
        t
    }

    /// Tool radius (half of diameter).
    pub fn radius(&self) -> f64 {
        self.diameter * 0.5
    }
}

// ============================================================
// Toolpath Point
// ============================================================

/// A point in a toolpath with optional feed rate override.
#[derive(Debug, Clone)]
pub struct ToolpathPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// True = rapid move (G00), False = feed move (G01).
    pub rapid: bool,
}

impl ToolpathPoint {
    /// Create a feed move point (G01).
    pub fn feed(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z, rapid: false }
    }

    /// Create a rapid move point (G00).
    pub fn rapid(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z, rapid: true }
    }
}

// ============================================================
// CAM Operation
// ============================================================

/// A CAM operation (contour, pocket, or drill).
#[derive(Debug, Clone)]
pub enum CamOperation {
    /// Contour milling: follow a 2D profile at a given depth.
    Contour {
        /// 2D profile points (x, y) — the path to follow.
        profile: Vec<(f64, f64)>,
        /// Z depth (negative = below surface).
        depth: f64,
        /// Z safe height for rapid moves between operations.
        safe_z: f64,
        /// Tool to use.
        tool: Tool,
        /// Step-down (Z increment per pass). 0 = single pass.
        step_down: f64,
    },

    /// Pocket milling: clear a rectangular pocket.
    PocketRect {
        /// Pocket center X.
        cx: f64,
        /// Pocket center Y.
        cy: f64,
        /// Pocket width (X).
        width: f64,
        /// Pocket height (Y).
        height: f64,
        /// Pocket depth (positive = depth below surface).
        depth: f64,
        /// Safe Z height.
        safe_z: f64,
        /// Tool.
        tool: Tool,
        /// Stepover ratio (0..1, fraction of tool diameter).
        stepover: f64,
        /// Step-down (Z per pass).
        step_down: f64,
    },

    /// Drilling: peck-drill cycle at specified locations.
    Drill {
        /// Hole positions (x, y).
        positions: Vec<(f64, f64)>,
        /// Hole depth (positive = depth below surface).
        depth: f64,
        /// Safe Z height.
        safe_z: f64,
        /// Tool (drill).
        tool: Tool,
        /// Peck depth (retract after this much).
        peck_depth: f64,
    },
}

impl CamOperation {
    /// Generate the toolpath points for this operation.
    pub fn generate_toolpath(&self) -> Result<Vec<ToolpathPoint>, CamError> {
        match self {
            CamOperation::Contour { profile, depth, safe_z, tool, step_down } => {
                self.generate_contour(profile, *depth, *safe_z, tool, *step_down)
            }
            CamOperation::PocketRect { cx, cy, width, height, depth, safe_z, tool, stepover, step_down } => {
                self.generate_pocket_rect(*cx, *cy, *width, *height, *depth, *safe_z, tool, *stepover, *step_down)
            }
            CamOperation::Drill { positions, depth, safe_z, tool, peck_depth } => {
                self.generate_drill(positions, *depth, *safe_z, tool, *peck_depth)
            }
        }
    }

    /// Generate contour milling toolpath.
    fn generate_contour(
        &self,
        profile: &[(f64, f64)],
        depth: f64,
        safe_z: f64,
        tool: &Tool,
        step_down: f64,
    ) -> Result<Vec<ToolpathPoint>, CamError> {
        if profile.len() < 2 {
            return Err(CamError::EmptyToolpath);
        }

        let mut points = Vec::new();
        let effective_step = if step_down > 0.0 { step_down } else { depth.abs() };
        let n_passes = ((depth.abs() / effective_step).ceil() as usize).max(1);
        let actual_step = depth.abs() / n_passes as f64;

        // Rapid to safe Z above first point
        let (start_x, start_y) = profile[0];
        points.push(ToolpathPoint::rapid(start_x, start_y, safe_z));

        for pass in 1..=n_passes {
            let z = -actual_step * pass as f64;

            // Plunge to depth at first point
            points.push(ToolpathPoint::feed(start_x, start_y, z));

            // Follow the profile
            for &(x, y) in profile.iter().skip(1) {
                points.push(ToolpathPoint::feed(x, y, z));
            }

            // Close the loop if profile is closed (first ≈ last check)
            let (last_x, last_y) = *profile.last().unwrap();
            let dx = last_x - start_x;
            let dy = last_y - start_y;
            if (dx * dx + dy * dy).sqrt() > 1e-6 {
                points.push(ToolpathPoint::feed(start_x, start_y, z));
            }

            // Rapid to safe Z
            points.push(ToolpathPoint::rapid(start_x, start_y, safe_z));
        }

        Ok(points)
    }

    /// Generate rectangular pocket milling toolpath (spiral inward).
    fn generate_pocket_rect(
        &self,
        cx: f64,
        cy: f64,
        width: f64,
        height: f64,
        depth: f64,
        safe_z: f64,
        tool: &Tool,
        stepover: f64,
        step_down: f64,
    ) -> Result<Vec<ToolpathPoint>, CamError> {
        if !(0.0..=1.0).contains(&stepover) {
            return Err(CamError::InvalidStepover(stepover));
        }

        let mut points = Vec::new();
        let effective_step_down = if step_down > 0.0 { step_down } else { depth };
        let n_z_passes = ((depth / effective_step_down).ceil() as usize).max(1);
        let actual_z_step = depth / n_z_passes as f64;
        let step = tool.diameter * stepover;

        let half_w = width * 0.5 - tool.radius();
        let half_h = height * 0.5 - tool.radius();

        if half_w <= 0.0 || half_h <= 0.0 {
            return Err(CamError::InvalidDepth(width));
        }

        // Start at pocket center
        points.push(ToolpathPoint::rapid(cx, cy, safe_z));

        for pass in 1..=n_z_passes {
            let z = -actual_z_step * pass as f64;

            // Plunge at center
            points.push(ToolpathPoint::feed(cx, cy, z));

            // Spiral outward from center to edges
            let n_steps_x = (half_w / step).ceil() as usize;
            let n_steps_y = (half_h / step).ceil() as usize;
            let n_steps = n_steps_x.max(n_steps_y);

            for i in 1..=n_steps {
                let fx = (i as f64 * step).min(half_w);
                let fy = (i as f64 * step).min(half_h);

                // Rectangle at this offset (clockwise)
                points.push(ToolpathPoint::feed(cx + fx, cy + fy, z));
                points.push(ToolpathPoint::feed(cx - fx, cy + fy, z));
                points.push(ToolpathPoint::feed(cx - fx, cy - fy, z));
                points.push(ToolpathPoint::feed(cx + fx, cy - fy, z));
                points.push(ToolpathPoint::feed(cx + fx, cy + fy, z));
            }

            // Return to safe Z
            points.push(ToolpathPoint::rapid(cx, cy, safe_z));
        }

        Ok(points)
    }

    /// Generate drilling toolpath (peck drill cycle).
    fn generate_drill(
        &self,
        positions: &[(f64, f64)],
        depth: f64,
        safe_z: f64,
        tool: &Tool,
        peck_depth: f64,
    ) -> Result<Vec<ToolpathPoint>, CamError> {
        if positions.is_empty() {
            return Err(CamError::EmptyToolpath);
        }

        let mut points = Vec::new();
        let effective_peck = if peck_depth > 0.0 { peck_depth } else { depth };
        let n_pecks = ((depth / effective_peck).ceil() as usize).max(1);
        let actual_peck = depth / n_pecks as f64;

        for &(x, y) in positions {
            // Rapid to safe Z above hole
            points.push(ToolpathPoint::rapid(x, y, safe_z));

            // Peck drill cycle
            for peck in 1..=n_pecks {
                let z = -actual_peck * peck as f64;
                // Plunge
                points.push(ToolpathPoint::feed(x, y, z));
                // Retract to safe Z (clear chips)
                points.push(ToolpathPoint::rapid(x, y, safe_z));
            }
        }

        Ok(points)
    }
}

// ============================================================
// G-code Generation
// ============================================================

/// G-code generator: converts toolpath points to G-code text.
pub struct GcodeGenerator {
    /// Program number (O0001, O0002, ...).
    pub program_number: u32,
    /// Program comment.
    pub comment: String,
}

impl Default for GcodeGenerator {
    fn default() -> Self {
        Self::new()
    }
}

impl GcodeGenerator {
    pub fn new() -> Self {
        Self {
            program_number: 1,
            comment: "Generated by 3Draper CAM".to_string(),
        }
    }

    /// Generate G-code from a list of operations.
    pub fn generate(&self, operations: &[CamOperation]) -> Result<String, CamError> {
        if operations.is_empty() {
            return Err(CamError::EmptyToolpath);
        }

        let mut gcode = String::new();

        // Program header
        writeln!(gcode, "O{:04}", self.program_number).ok();
        writeln!(gcode, "({})", self.comment).ok();
        writeln!(gcode, "(Generated by 3Draper CAM)").ok();
        writeln!(gcode).ok();

        // Safety block
        writeln!(gcode, "G17 G21 G40 G49 G80 G90").ok(); // Safe startup
        writeln!(gcode).ok();

        let mut current_tool: Option<u32> = None;
        let mut first_op = true;

        for (i, op) in operations.iter().enumerate() {
            let tool = match op {
                CamOperation::Contour { tool, .. }
                | CamOperation::PocketRect { tool, .. }
                | CamOperation::Drill { tool, .. } => tool,
            };

            // Tool change if needed
            if current_tool != Some(tool.number) {
                if !first_op {
                    writeln!(gcode, "M05").ok(); // Spindle off
                    writeln!(gcode, "G00 Z50.0").ok(); // Retract
                }
                writeln!(gcode, "T{:02}", tool.number).ok();
                writeln!(gcode, "M06").ok(); // Tool change
                writeln!(gcode, "M03 S{:.0}", tool.spindle_rpm).ok(); // Spindle on
                writeln!(gcode, "F{:.0}", tool.feed_rate).ok(); // Feed rate
                current_tool = Some(tool.number);
            }

            writeln!(gcode).ok();
            writeln!(gcode, "(Operation {} — {})", i + 1, self.op_name(op)).ok();
            writeln!(gcode).ok();

            // Generate toolpath points
            let toolpath = op.generate_toolpath()?;

            // Convert points to G-code
            for pt in &toolpath {
                if pt.rapid {
                    writeln!(gcode, "G00 X{:.3} Y{:.3} Z{:.3}", pt.x, pt.y, pt.z).ok();
                } else {
                    writeln!(gcode, "G01 X{:.3} Y{:.3} Z{:.3}", pt.x, pt.y, pt.z).ok();
                }
            }

            first_op = false;
        }

        // Program footer
        writeln!(gcode).ok();
        writeln!(gcode, "M05").ok(); // Spindle off
        writeln!(gcode, "G00 Z50.0").ok(); // Retract to safe Z
        writeln!(gcode, "G00 X0 Y0").ok(); // Return to origin
        writeln!(gcode, "M30").ok(); // Program end
        writeln!(gcode, "%").ok(); // EOF marker

        log::info!(
            "G-code: generated {} operations, program O{:04}",
            operations.len(),
            self.program_number
        );

        Ok(gcode)
    }

    /// Get a human-readable name for an operation.
    fn op_name(&self, op: &CamOperation) -> String {
        match op {
            CamOperation::Contour { .. } => "Contour Milling".to_string(),
            CamOperation::PocketRect { width, height, .. } => {
                format!("Pocket {:.0}×{:.0}mm", width, height)
            }
            CamOperation::Drill { positions, .. } => {
                format!("Drilling ({} holes)", positions.len())
            }
        }
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ---------- Tool tests ----------

    #[test]
    fn test_tool_creation() {
        let tool = Tool::new(1, ToolType::EndMill, 6.0).unwrap();
        assert_eq!(tool.number, 1);
        assert_eq!(tool.diameter, 6.0);
        assert_relative_eq!(tool.radius(), 3.0);
    }

    #[test]
    fn test_tool_invalid_diameter() {
        assert!(Tool::new(1, ToolType::EndMill, 0.0).is_err());
        assert!(Tool::new(1, ToolType::EndMill, -1.0).is_err());
    }

    #[test]
    fn test_tool_presets() {
        let t = Tool::endmill_6mm();
        assert_eq!(t.diameter, 6.0);
        assert_eq!(t.tool_type, ToolType::EndMill);

        let d = Tool::drill_5mm();
        assert_eq!(d.diameter, 5.0);
        assert_eq!(d.tool_type, ToolType::Drill);
    }

    #[test]
    fn test_tool_type_name() {
        assert_eq!(ToolType::EndMill.name(), "End Mill");
        assert_eq!(ToolType::Drill.name(), "Drill");
    }

    // ---------- Contour tests ----------

    #[test]
    fn test_contour_toolpath() {
        let tool = Tool::endmill_6mm();
        let profile = vec![(0.0, 0.0), (50.0, 0.0), (50.0, 30.0), (0.0, 30.0), (0.0, 0.0)];
        let op = CamOperation::Contour {
            profile,
            depth: 5.0,
            safe_z: 10.0,
            tool,
            step_down: 2.5, // 2 passes
        };

        let toolpath = op.generate_toolpath().unwrap();
        assert!(!toolpath.is_empty());
        // 1 initial rapid + 2 passes × (1 plunge + 4 feed + 1 rapid) = 1 + 2×6 = 13
        assert!(toolpath.len() >= 13, "Expected >= 13 points, got {}", toolpath.len());
        // Verify first point is rapid (safe Z)
        assert!(toolpath[0].rapid);
    }

    #[test]
    fn test_contour_single_pass() {
        let tool = Tool::endmill_3mm();
        let profile = vec![(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)];
        let op = CamOperation::Contour {
            profile,
            depth: 3.0,
            safe_z: 5.0,
            tool,
            step_down: 0.0, // single pass
        };

        let toolpath = op.generate_toolpath().unwrap();
        assert!(!toolpath.is_empty());
    }

    #[test]
    fn test_contour_empty_profile() {
        let tool = Tool::endmill_6mm();
        let op = CamOperation::Contour {
            profile: vec![],
            depth: 5.0,
            safe_z: 10.0,
            tool,
            step_down: 2.0,
        };
        assert!(op.generate_toolpath().is_err());
    }

    // ---------- Pocket tests ----------

    #[test]
    fn test_pocket_rect_toolpath() {
        let tool = Tool::endmill_6mm();
        let op = CamOperation::PocketRect {
            cx: 50.0,
            cy: 50.0,
            width: 40.0,
            height: 30.0,
            depth: 5.0,
            safe_z: 10.0,
            tool,
            stepover: 0.5,
            step_down: 2.5,
        };

        let toolpath = op.generate_toolpath().unwrap();
        assert!(!toolpath.is_empty());
        // Should have points for 2 Z-passes
        assert!(toolpath.len() > 10);
    }

    #[test]
    fn test_pocket_invalid_stepover() {
        let tool = Tool::endmill_6mm();
        let op = CamOperation::PocketRect {
            cx: 0.0, cy: 0.0, width: 40.0, height: 30.0,
            depth: 5.0, safe_z: 10.0, tool,
            stepover: 1.5, // invalid
            step_down: 2.0,
        };
        assert!(op.generate_toolpath().is_err());
    }

    // ---------- Drill tests ----------

    #[test]
    fn test_drill_toolpath() {
        let tool = Tool::drill_5mm();
        let op = CamOperation::Drill {
            positions: vec![(10.0, 10.0), (50.0, 10.0), (50.0, 50.0)],
            depth: 10.0,
            safe_z: 5.0,
            tool,
            peck_depth: 3.0, // ~4 pecks
        };

        let toolpath = op.generate_toolpath().unwrap();
        // 3 holes × (1 rapid + 4 pecks × (plunge + retract))
        assert!(toolpath.len() > 20);
    }

    #[test]
    fn test_drill_no_positions() {
        let tool = Tool::drill_5mm();
        let op = CamOperation::Drill {
            positions: vec![],
            depth: 10.0,
            safe_z: 5.0,
            tool,
            peck_depth: 3.0,
        };
        assert!(op.generate_toolpath().is_err());
    }

    // ---------- G-code tests ----------

    #[test]
    fn test_gcode_generation() {
        let tool = Tool::endmill_6mm();
        let op = CamOperation::Contour {
            profile: vec![(0.0, 0.0), (50.0, 0.0), (50.0, 30.0), (0.0, 30.0)],
            depth: 5.0,
            safe_z: 10.0,
            tool,
            step_down: 0.0,
        };

        let gen = GcodeGenerator::new();
        let gcode = gen.generate(&[op]).unwrap();

        assert!(gcode.contains("O0001"));
        assert!(gcode.contains("G00")); // Rapid
        assert!(gcode.contains("G01")); // Feed
        assert!(gcode.contains("M03")); // Spindle on
        assert!(gcode.contains("M05")); // Spindle off
        assert!(gcode.contains("M30")); // Program end
        assert!(gcode.contains("M06")); // Tool change
    }

    #[test]
    fn test_gcode_multiple_operations() {
        let tool1 = Tool::endmill_6mm();
        let tool2 = Tool::drill_5mm();

        let ops = vec![
            CamOperation::Contour {
                profile: vec![(0.0, 0.0), (50.0, 0.0), (50.0, 50.0)],
                depth: 3.0, safe_z: 10.0, tool: tool1, step_down: 0.0,
            },
            CamOperation::Drill {
                positions: vec![(25.0, 25.0)],
                depth: 10.0, safe_z: 10.0, tool: tool2, peck_depth: 5.0,
            },
        ];

        let gen = GcodeGenerator::new();
        let gcode = gen.generate(&ops).unwrap();

        // Should have tool change between operations
        assert!(gcode.contains("T01"));
        assert!(gcode.contains("T04"));
        assert!(gcode.contains("M06")); // Tool change
        assert!(gcode.contains("(Operation 1"));
        assert!(gcode.contains("(Operation 2"));
    }

    #[test]
    fn test_gcode_empty_operations() {
        let gen = GcodeGenerator::new();
        let result = gen.generate(&[]);
        assert!(result.is_err());
    }

    #[test]
    fn test_gcode_has_safety_block() {
        let tool = Tool::endmill_6mm();
        let op = CamOperation::Contour {
            profile: vec![(0.0, 0.0), (10.0, 0.0)],
            depth: 1.0, safe_z: 5.0, tool, step_down: 0.0,
        };
        let gen = GcodeGenerator::new();
        let gcode = gen.generate(&[op]).unwrap();
        // Safety block: G17 G21 G40 G49 G80 G90
        assert!(gcode.contains("G17 G21 G40 G49 G80 G90"));
    }

    #[test]
    fn test_gcode_feed_rate() {
        let mut tool = Tool::endmill_6mm();
        tool.feed_rate = 750.0;
        let op = CamOperation::Contour {
            profile: vec![(0.0, 0.0), (10.0, 0.0)],
            depth: 1.0, safe_z: 5.0, tool, step_down: 0.0,
        };
        let gen = GcodeGenerator::new();
        let gcode = gen.generate(&[op]).unwrap();
        assert!(gcode.contains("F750"));
    }

    #[test]
    fn test_toolpath_point_rapid_vs_feed() {
        let rapid_pt = ToolpathPoint::rapid(10.0, 20.0, 5.0);
        assert!(rapid_pt.rapid);

        let feed_pt = ToolpathPoint::feed(10.0, 20.0, -5.0);
        assert!(!feed_pt.rapid);
    }
}
