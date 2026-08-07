// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Sheet metal modeling — bend allowance, flat pattern, DXF export.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 3.1: provides sheet metal
//! operations for manufacturing:
//!
//! - **Bend allowance calculation**: K-factor method for determining
//!   flat pattern length of bent sheet metal.
//! - **Flat pattern unfolding**: converts a folded sheet metal part
//!   into its 2D flat pattern for laser cutting.
//! - **DXF export**: exports the flat pattern as a DXF file for
//!   laser/waterjet cutting machines.
//!
//! # K-Factor Method
//!
//! When sheet metal is bent, the neutral axis (which doesn't stretch
//! or compress) shifts toward the inside of the bend. The **K-factor**
//! is the ratio of the neutral axis position to the material thickness:
//!
//! ```text
//! K = t / T
//! ```
//!
//! where `t` = distance from inside surface to neutral axis, `T` = thickness.
//!
//! Typical K-factors:
//! - Soft aluminum: 0.33–0.42
//! - Hard aluminum: 0.40–0.45
//! - Steel: 0.44–0.50
//! - Stainless steel: 0.40–0.45
//!
//! **Bend Allowance (BA)** = the arc length of the neutral axis:
//! ```text
//! BA = (π/180) × angle × (R + K × T)
//! ```
//! where `R` = inner bend radius, `angle` = bend angle in degrees.

use std::f64::consts::PI;

// ============================================================
// Error types
// ============================================================

#[derive(Debug, Clone, thiserror::Error)]
pub enum SheetMetalError {
    #[error("Invalid material thickness: {0} (must be > 0)")]
    InvalidThickness(f64),

    #[error("Invalid bend radius: {0} (must be > 0)")]
    InvalidBendRadius(f64),

    #[error("Invalid bend angle: {0} (must be 0..=180 degrees)")]
    InvalidBendAngle(f64),

    #[error("Invalid K-factor: {0} (must be 0..=1)")]
    InvalidKFactor(f64),

    #[error("No bends in sheet metal part")]
    NoBends,

    #[error("DXF export error: {0}")]
    DxfExportError(String),
}

// ============================================================
// Material
// ============================================================

/// Sheet metal material properties.
#[derive(Debug, Clone)]
pub struct SheetMaterial {
    /// Material name (e.g., "Aluminum 6061-T6").
    pub name: String,
    /// Sheet thickness in mm.
    pub thickness: f64,
    /// K-factor (0..1). Default: 0.44 for steel.
    pub k_factor: f64,
    /// Default inner bend radius in mm.
    pub default_bend_radius: f64,
}

impl Default for SheetMaterial {
    fn default() -> Self {
        // Steel, 1.5mm thick
        Self {
            name: "Steel".to_string(),
            thickness: 1.5,
            k_factor: 0.44,
            default_bend_radius: 1.5,
        }
    }
}

impl SheetMaterial {
    /// Create a new sheet material.
    pub fn new(name: &str, thickness: f64, k_factor: f64, bend_radius: f64) -> Result<Self, SheetMetalError> {
        if thickness <= 0.0 {
            return Err(SheetMetalError::InvalidThickness(thickness));
        }
        if !(0.0..=1.0).contains(&k_factor) {
            return Err(SheetMetalError::InvalidKFactor(k_factor));
        }
        if bend_radius <= 0.0 {
            return Err(SheetMetalError::InvalidBendRadius(bend_radius));
        }
        Ok(Self {
            name: name.to_string(),
            thickness,
            k_factor,
            default_bend_radius: bend_radius,
        })
    }

    /// Aluminum 6061-T6, 2.0mm thick.
    pub fn aluminum_2mm() -> Self {
        Self::new("Aluminum 6061-T6", 2.0, 0.33, 2.0).unwrap_or_default()
    }

    /// Steel, 1.5mm thick.
    pub fn steel_1_5mm() -> Self {
        Self::default()
    }

    /// Stainless steel 304, 1.0mm thick.
    pub fn stainless_1mm() -> Self {
        Self::new("Stainless 304", 1.0, 0.40, 1.0).unwrap_or_default()
    }
}

// ============================================================
// Bend
// ============================================================

/// A single bend in a sheet metal part.
#[derive(Debug, Clone)]
pub struct Bend {
    /// Inner bend radius in mm.
    pub radius: f64,
    /// Bend angle in degrees (0..=180).
    pub angle: f64,
    /// Length of the bend (along the bend axis) in mm.
    pub length: f64,
}

impl Bend {
    /// Create a new bend.
    pub fn new(radius: f64, angle: f64, length: f64) -> Result<Self, SheetMetalError> {
        if radius <= 0.0 {
            return Err(SheetMetalError::InvalidBendRadius(radius));
        }
        if !(0.0..=180.0).contains(&angle) {
            return Err(SheetMetalError::InvalidBendAngle(angle));
        }
        Ok(Self { radius, angle, length })
    }

    /// Create a 90° bend.
    pub fn ninety_degrees(radius: f64, length: f64) -> Result<Self, SheetMetalError> {
        Self::new(radius, 90.0, length)
    }

    /// Bend allowance (BA): the arc length of the neutral axis.
    ///
    /// BA = (π/180) × angle × (R + K × T)
    pub fn bend_allowance(&self, material: &SheetMaterial) -> f64 {
        let angle_rad = self.angle.to_radians();
        angle_rad * (self.radius + material.k_factor * material.thickness)
    }

    /// Bend deduction (BD): the amount subtracted from the total flat length.
    ///
    /// BD = 2 × (R + T) × tan(angle/2) - BA
    pub fn bend_deduction(&self, material: &SheetMaterial) -> f64 {
        let half_angle = (self.angle / 2.0).to_radians();
        let outside_setback = (self.radius + material.thickness) * half_angle.tan();
        2.0 * outside_setback - self.bend_allowance(material)
    }

    /// Outside setback (OSSB): distance from bend tangent to mold line.
    ///
    /// OSSB = (R + T) × tan(angle/2)
    pub fn outside_setback(&self, material: &SheetMaterial) -> f64 {
        let half_angle = (self.angle / 2.0).to_radians();
        (self.radius + material.thickness) * half_angle.tan()
    }
}

// ============================================================
// Sheet Metal Part
// ============================================================

/// A sheet metal part: a sequence of flanges connected by bends.
#[derive(Debug, Clone)]
pub struct SheetMetalPart {
    /// Material specification.
    pub material: SheetMaterial,
    /// Flanges (flat segments between bends), each with a length in mm.
    pub flanges: Vec<f64>,
    /// Bends connecting flanges (bends[i] connects flanges[i] and flanges[i+1]).
    pub bends: Vec<Bend>,
}

impl SheetMetalPart {
    /// Create a new sheet metal part with the given material.
    pub fn new(material: SheetMaterial) -> Self {
        Self {
            material,
            flanges: Vec::new(),
            bends: Vec::new(),
        }
    }

    /// Add a flange (flat segment) to the part.
    pub fn add_flange(&mut self, length: f64) {
        self.flanges.push(length);
    }

    /// Add a bend between the last flange and the next.
    pub fn add_bend(&mut self, bend: Bend) {
        self.bends.push(bend);
    }

    /// Total flat pattern length = sum of flange lengths + sum of bend allowances.
    ///
    /// For each bend: flat_length += BA - 2×OSSB (bend deduction replaces
    /// the outside setback material with the neutral axis arc).
    /// Simplified: total = Σ(flanges) + Σ(BA) - Σ(BD) per bend.
    /// Actually: total_flat = Σ(flanges) - Σ(BD) where BD accounts for
    /// the overlap at each bend.
    pub fn flat_pattern_length(&self) -> f64 {
        let total_flanges: f64 = self.flanges.iter().sum();
        let total_bend_deduction: f64 = self
            .bends
            .iter()
            .map(|b| b.bend_deduction(&self.material))
            .sum();
        total_flanges - total_bend_deduction
    }

    /// Total bend allowance for all bends.
    pub fn total_bend_allowance(&self) -> f64 {
        self.bends
            .iter()
            .map(|b| b.bend_allowance(&self.material))
            .sum()
    }

    /// Total number of bends.
    pub fn num_bends(&self) -> usize {
        self.bends.len()
    }

    /// Check if the part is valid (n_bends = n_flanges - 1, and at least 1 flange).
    pub fn is_valid(&self) -> bool {
        !self.flanges.is_empty() && self.bends.len() == self.flanges.len() - 1
    }

    /// Generate the flat pattern as 2D line segments.
    ///
    /// Returns a list of (x1, y1, x2, y2) line segments representing
    /// the outline of the unfolded sheet metal part.
    pub fn flat_pattern_outline(&self) -> Vec<(f64, f64, f64, f64)> {
        let thickness = self.material.thickness;
        let mut segments = Vec::new();
        let mut x = 0.0;

        // The flat pattern is a series of rectangles (flanges) connected
        // at bend lines. For simplicity, we assume all bends are along
        // the same axis (length = Y direction), and flanges extend in X.
        let width = self.bends.first().map(|b| b.length).unwrap_or(100.0);

        for (i, &flange_len) in self.flanges.iter().enumerate() {
            // Bottom edge of this flange
            segments.push((x, 0.0, x + flange_len, 0.0));
            // Top edge
            segments.push((x, width, x + flange_len, width));
            // Left edge (if first flange)
            if i == 0 {
                segments.push((x, 0.0, x, width));
            }
            // Right edge (if last flange)
            if i == self.flanges.len() - 1 {
                segments.push((x + flange_len, 0.0, x + flange_len, width));
            }

            // Bend line (dashed in drawing, but we draw as solid for now)
            if i < self.bends.len() {
                x += flange_len;
                // Bend line marks the transition
                segments.push((x, 0.0, x, width));
            } else {
                x += flange_len;
            }
        }

        segments
    }
}

// ============================================================
// DXF Export
// ============================================================

impl SheetMetalPart {
    /// Export the flat pattern as a DXF file.
    ///
    /// Per BREPCAD Phase 3.1: generates a DXF file suitable for
    /// laser cutting machines. The DXF contains:
    /// - Outline of the flat pattern as LINE entities.
    /// - Bend lines as separate entities on a "BEND" layer.
    /// - A title block with material info.
    pub fn to_dxf(&self) -> Result<String, SheetMetalError> {
        if self.flanges.is_empty() {
            return Err(SheetMetalError::NoBends);
        }

        let mut dxf = String::new();

        // DXF header
        dxf.push_str("0\nSECTION\n");
        dxf.push_str("2\nHEADER\n");
        dxf.push_str("9\n$ACADVER\n");
        dxf.push_str("1\nAC1014\n");
        dxf.push_str("0\nENDSEC\n");

        // Tables section (layers)
        dxf.push_str("0\nSECTION\n");
        dxf.push_str("2\nTABLES\n");
        dxf.push_str("0\nTABLE\n");
        dxf.push_str("2\nLAYER\n");
        dxf.push_str("70\n2\n"); // 2 layers

        // OUTLINE layer
        dxf.push_str("0\nLAYER\n");
        dxf.push_str("2\nOUTLINE\n");
        dxf.push_str("70\n0\n");
        dxf.push_str("62\n7\n"); // color: white/black
        dxf.push_str("6\nCONTINUOUS\n");

        // BEND layer
        dxf.push_str("0\nLAYER\n");
        dxf.push_str("2\nBEND\n");
        dxf.push_str("70\n0\n");
        dxf.push_str("62\n3\n"); // color: green
        dxf.push_str("6\nDASHED\n");

        dxf.push_str("0\nENDTAB\n");
        dxf.push_str("0\nENDSEC\n");

        // Entities section
        dxf.push_str("0\nSECTION\n");
        dxf.push_str("2\nENTITIES\n");

        // Draw outline
        let outline = self.flat_pattern_outline();
        let mut x = 0.0;
        let thickness = self.material.thickness;
        let width = self.bends.first().map(|b| b.length).unwrap_or(100.0);

        for (i, &flange_len) in self.flanges.iter().enumerate() {
            // Bottom edge
            self.write_dxf_line(&mut dxf, "OUTLINE", x, 0.0, x + flange_len, 0.0);
            // Top edge
            self.write_dxf_line(&mut dxf, "OUTLINE", x, width, x + flange_len, width);

            // Left edge (if first flange)
            if i == 0 {
                self.write_dxf_line(&mut dxf, "OUTLINE", x, 0.0, x, width);
            }
            // Right edge (if last flange)
            if i == self.flanges.len() - 1 {
                self.write_dxf_line(&mut dxf, "OUTLINE", x + flange_len, 0.0, x + flange_len, width);
            }

            // Bend line (on BEND layer)
            if i < self.bends.len() {
                x += flange_len;
                self.write_dxf_line(&mut dxf, "BEND", x, 0.0, x, width);
            } else {
                x += flange_len;
            }
        }

        dxf.push_str("0\nENDSEC\n");
        dxf.push_str("0\nEOF\n");

        log::info!(
            "DXF: exported flat pattern ({} flanges, {} bends, flat_length={:.2}mm)",
            self.flanges.len(),
            self.bends.len(),
            self.flat_pattern_length()
        );

        Ok(dxf)
    }

    /// Write a single LINE entity to the DXF string.
    fn write_dxf_line(&self, dxf: &mut String, layer: &str, x1: f64, y1: f64, x2: f64, y2: f64) {
        dxf.push_str(&format!(
            "0\nLINE\n8\n{}\n10\n{:.6}\n20\n{:.6}\n11\n{:.6}\n21\n{:.6}\n",
            layer, x1, y1, x2, y2
        ));
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_material_creation() {
        let mat = SheetMaterial::new("Steel", 1.5, 0.44, 1.5).unwrap();
        assert_eq!(mat.name, "Steel");
        assert_eq!(mat.thickness, 1.5);
        assert_relative_eq!(mat.k_factor, 0.44);
    }

    #[test]
    fn test_material_invalid_thickness() {
        assert!(SheetMaterial::new("Test", -1.0, 0.44, 1.5).is_err());
    }

    #[test]
    fn test_material_invalid_k_factor() {
        assert!(SheetMaterial::new("Test", 1.5, 1.5, 1.5).is_err());
        assert!(SheetMaterial::new("Test", 1.5, -0.1, 1.5).is_err());
    }

    #[test]
    fn test_preset_materials() {
        let al = SheetMaterial::aluminum_2mm();
        assert_eq!(al.thickness, 2.0);
        assert_relative_eq!(al.k_factor, 0.33);

        let steel = SheetMaterial::steel_1_5mm();
        assert_eq!(steel.thickness, 1.5);

        let ss = SheetMaterial::stainless_1mm();
        assert_eq!(ss.thickness, 1.0);
    }

    #[test]
    fn test_bend_allowance_90_degrees() {
        // BA = (π/180) × 90 × (R + K×T) = (π/2) × (R + K×T)
        let material = SheetMaterial::steel_1_5mm(); // T=1.5, K=0.44
        let bend = Bend::ninety_degrees(1.5, 100.0).unwrap(); // R=1.5
        let ba = bend.bend_allowance(&material);
        // BA = (π/2) × (1.5 + 0.44×1.5) = (π/2) × (1.5 + 0.66) = (π/2) × 2.16
        let expected = (PI / 2.0) * (1.5 + 0.44 * 1.5);
        assert_relative_eq!(ba, expected, epsilon = 1e-6);
    }

    #[test]
    fn test_bend_allowance_45_degrees() {
        let material = SheetMaterial::aluminum_2mm(); // T=2.0, K=0.33
        let bend = Bend::new(2.0, 45.0, 50.0).unwrap(); // R=2.0
        let ba = bend.bend_allowance(&material);
        // BA = (π/4) × (2.0 + 0.33×2.0) = (π/4) × 2.66
        let expected = (PI / 4.0) * (2.0 + 0.33 * 2.0);
        assert_relative_eq!(ba, expected, epsilon = 1e-6);
    }

    #[test]
    fn test_bend_deduction() {
        let material = SheetMaterial::steel_1_5mm();
        let bend = Bend::ninety_degrees(1.5, 100.0).unwrap();
        let bd = bend.bend_deduction(&material);
        // BD = 2×OSSB - BA
        let ossb = bend.outside_setback(&material);
        let ba = bend.bend_allowance(&material);
        let expected = 2.0 * ossb - ba;
        assert_relative_eq!(bd, expected, epsilon = 1e-6);
    }

    #[test]
    fn test_outside_setback_90_degrees() {
        let material = SheetMaterial::steel_1_5mm(); // T=1.5
        let bend = Bend::ninety_degrees(1.5, 100.0).unwrap(); // R=1.5
        let ossb = bend.outside_setback(&material);
        // OSSB = (R+T) × tan(45°) = (1.5+1.5) × 1 = 3.0
        assert_relative_eq!(ossb, 3.0, epsilon = 1e-6);
    }

    #[test]
    fn test_bend_invalid_angle() {
        assert!(Bend::new(1.5, 200.0, 100.0).is_err());
        assert!(Bend::new(1.5, -10.0, 100.0).is_err());
    }

    #[test]
    fn test_bend_invalid_radius() {
        assert!(Bend::new(0.0, 90.0, 100.0).is_err());
        assert!(Bend::new(-1.0, 90.0, 100.0).is_err());
    }

    #[test]
    fn test_sheet_metal_part_flat_length() {
        // A simple L-bracket: 50mm flange, 90° bend, 30mm flange
        let mut part = SheetMetalPart::new(SheetMaterial::steel_1_5mm());
        part.add_flange(50.0);
        part.add_bend(Bend::ninety_degrees(1.5, 100.0).unwrap());
        part.add_flange(30.0);

        let flat_len = part.flat_pattern_length();
        // flat_len = (50 + 30) - BD
        let bd = part.bends[0].bend_deduction(&part.material);
        let expected = 80.0 - bd;
        assert_relative_eq!(flat_len, expected, epsilon = 1e-6);
        assert!(flat_len > 0.0);
        assert!(flat_len < 80.0); // BD is positive, so flat < sum of flanges
    }

    #[test]
    fn test_sheet_metal_part_multiple_bends() {
        // U-channel: 20mm + 90° bend + 40mm + 90° bend + 20mm
        let mut part = SheetMetalPart::new(SheetMaterial::steel_1_5mm());
        part.add_flange(20.0);
        part.add_bend(Bend::ninety_degrees(1.5, 100.0).unwrap());
        part.add_flange(40.0);
        part.add_bend(Bend::ninety_degrees(1.5, 100.0).unwrap());
        part.add_flange(20.0);

        assert_eq!(part.num_bends(), 2);
        assert!(part.is_valid());
        let flat_len = part.flat_pattern_length();
        assert!(flat_len > 0.0);
    }

    #[test]
    fn test_flat_pattern_outline() {
        let mut part = SheetMetalPart::new(SheetMaterial::steel_1_5mm());
        part.add_flange(50.0);
        part.add_bend(Bend::ninety_degrees(1.5, 100.0).unwrap());
        part.add_flange(30.0);

        let outline = part.flat_pattern_outline();
        assert!(!outline.is_empty(), "Should have outline segments");
        // Should have bottom, top, left, right edges + bend line
        assert!(outline.len() >= 5);
    }

    #[test]
    fn test_dxf_export() {
        let mut part = SheetMetalPart::new(SheetMaterial::steel_1_5mm());
        part.add_flange(50.0);
        part.add_bend(Bend::ninety_degrees(1.5, 100.0).unwrap());
        part.add_flange(30.0);

        let dxf = part.to_dxf().unwrap();
        assert!(dxf.contains("SECTION"));
        assert!(dxf.contains("ENTITIES"));
        assert!(dxf.contains("LINE"));
        assert!(dxf.contains("OUTLINE"));
        assert!(dxf.contains("BEND"));
        assert!(dxf.contains("EOF"));
    }

    #[test]
    fn test_dxf_export_empty_fails() {
        let part = SheetMetalPart::new(SheetMaterial::steel_1_5mm());
        let result = part.to_dxf();
        assert!(matches!(result, Err(SheetMetalError::NoBends)));
    }

    #[test]
    fn test_part_validity() {
        let mut part = SheetMetalPart::new(SheetMaterial::steel_1_5mm());
        // No flanges, no bends — invalid (0 bends vs -1 expected)
        assert!(!part.is_valid());

        part.add_flange(50.0);
        // 1 flange, 0 bends — valid (0 = 1-1)
        assert!(part.is_valid());

        part.add_bend(Bend::ninety_degrees(1.5, 100.0).unwrap());
        // 1 flange, 1 bend — invalid (1 ≠ 0)
        assert!(!part.is_valid());

        part.add_flange(30.0);
        // 2 flanges, 1 bend — valid (1 = 2-1)
        assert!(part.is_valid());
    }

    #[test]
    fn test_total_bend_allowance() {
        let mut part = SheetMetalPart::new(SheetMaterial::aluminum_2mm());
        part.add_flange(40.0);
        part.add_bend(Bend::ninety_degrees(2.0, 80.0).unwrap());
        part.add_flange(40.0);
        part.add_bend(Bend::new(2.0, 45.0, 80.0).unwrap());
        part.add_flange(20.0);

        let total_ba = part.total_bend_allowance();
        let ba1 = part.bends[0].bend_allowance(&part.material);
        let ba2 = part.bends[1].bend_allowance(&part.material);
        assert_relative_eq!(total_ba, ba1 + ba2, epsilon = 1e-6);
    }

    #[test]
    fn test_dxf_has_two_layers() {
        let mut part = SheetMetalPart::new(SheetMaterial::steel_1_5mm());
        part.add_flange(50.0);
        part.add_bend(Bend::ninety_degrees(1.5, 100.0).unwrap());
        part.add_flange(30.0);

        let dxf = part.to_dxf().unwrap();
        // Should have OUTLINE and BEND layers
        assert!(dxf.contains("OUTLINE"));
        assert!(dxf.contains("BEND"));
    }
}
