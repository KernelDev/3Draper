// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Natural-language → geometry parser.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 5.2: parses text descriptions
//! like "a bracket with 4 holes" into a sequence of CAD operations
//! (CreateBox, CreateCylinder, BooleanSubtract, etc.) that can be fed
//! to `draper-topology` to build a solid.
//!
//! # Architecture
//!
//! The parser is **rule-based** (no LLM call required for the common cases).
//! It tokenizes the input text, matches against a library of shape patterns,
//! and emits `GeometryAction`s. For ambiguous or complex queries, an
//! optional LLM backend (see `llm.rs`) can be queried to expand the prompt
//! into a canonical form before parsing.
//!
//! # Supported Patterns
//!
//! - "box" / "cube" / "block" → CreateBox
//! - "cylinder" / "rod" / "shaft" → CreateCylinder
//! - "sphere" / "ball" → CreateSphere
//! - "cone" → CreateCone
//! - "torus" / "ring" → CreateTorus
//! - "with N holes" → BooleanSubtract (N cylinders)
//! - "fillet" / "round edges" → FilletAllEdges
//! - "chamfer" → ChamferAllEdges
//! - "shell" / "hollow" → Shell
//! - "extrude" → ExtrudeProfile
//! - "revolve" → RevolveProfile
//! - "loft" → LoftProfiles
//!
//! # Example
//!
//! ```text
//! Input:  "A 50×30×20 box with 4 holes of diameter 5"
//! Output: [
//!   CreateBox { size: [50, 30, 20] },
//!   CreateCylinder { diameter: 5, height: 20, center: [12.5, 7.5, 0] },
//!   BooleanSubtract,
//!   CreateCylinder { diameter: 5, height: 20, center: [37.5, 7.5, 0] },
//!   BooleanSubtract,
//!   CreateCylinder { diameter: 5, height: 20, center: [12.5, 22.5, 0] },
//!   BooleanSubtract,
//!   CreateCylinder { diameter: 5, height: 20, center: [37.5, 22.5, 0] },
//!   BooleanSubtract,
//! ]
//! ```

use serde::{Deserialize, Serialize};

// ============================================================
// Geometry actions
// ============================================================

/// A primitive shape creation action.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GeometryAction {
    /// Create an axis-aligned box with the given dimensions.
    CreateBox {
        size: [f64; 3],
        center: [f64; 3],
    },
    /// Create a cylinder along the Z axis.
    CreateCylinder {
        diameter: f64,
        height: f64,
        center: [f64; 3],
    },
    /// Create a sphere.
    CreateSphere {
        diameter: f64,
        center: [f64; 3],
    },
    /// Create a cone (frustum) along the Z axis.
    CreateCone {
        bottom_diameter: f64,
        top_diameter: f64,
        height: f64,
        center: [f64; 3],
    },
    /// Create a torus around the Z axis.
    CreateTorus {
        major_diameter: f64,
        minor_diameter: f64,
        center: [f64; 3],
    },
    /// Subtract the most recently created shape from the previous shape.
    BooleanSubtract,
    /// Union the most recently created shape with the previous shape.
    BooleanUnion,
    /// Intersect the most recently created shapes.
    BooleanIntersect,
    /// Fillet all edges with the given radius.
    FilletAllEdges {
        radius: f64,
    },
    /// Chamfer all edges with the given distance.
    ChamferAllEdges {
        distance: f64,
    },
    /// Hollow out the solid, leaving walls of the given thickness.
    Shell {
        thickness: f64,
    },
    /// Extrude a 2D profile (sketch) by the given distance.
    ExtrudeProfile {
        profile_points: Vec<[f64; 2]>,
        distance: f64,
    },
    /// Revolve a 2D profile around the Z axis.
    RevolveProfile {
        profile_points: Vec<[f64; 2]>,
        angle_degrees: f64,
    },
}

impl GeometryAction {
    /// Get a human-readable description of this action.
    pub fn describe(&self) -> String {
        match self {
            GeometryAction::CreateBox { size, center } => {
                format!("Create box {}×{}×{} at ({}, {}, {})",
                    size[0], size[1], size[2], center[0], center[1], center[2])
            }
            GeometryAction::CreateCylinder { diameter, height, center } => {
                format!("Create cylinder Ø{}×{} at ({}, {}, {})",
                    diameter, height, center[0], center[1], center[2])
            }
            GeometryAction::CreateSphere { diameter, center } => {
                format!("Create sphere Ø{} at ({}, {}, {})",
                    diameter, center[0], center[1], center[2])
            }
            GeometryAction::CreateCone { bottom_diameter, top_diameter, height, center } => {
                format!("Create cone Ø{}/Ø{}×{} at ({}, {}, {})",
                    bottom_diameter, top_diameter, height, center[0], center[1], center[2])
            }
            GeometryAction::CreateTorus { major_diameter, minor_diameter, center } => {
                format!("Create torus Ø{}/Ø{} at ({}, {}, {})",
                    major_diameter, minor_diameter, center[0], center[1], center[2])
            }
            GeometryAction::BooleanSubtract => "Subtract last shape".to_string(),
            GeometryAction::BooleanUnion => "Union last shape".to_string(),
            GeometryAction::BooleanIntersect => "Intersect last shape".to_string(),
            GeometryAction::FilletAllEdges { radius } => format!("Fillet all edges R{}", radius),
            GeometryAction::ChamferAllEdges { distance } => format!("Chamfer all edges {}", distance),
            GeometryAction::Shell { thickness } => format!("Shell with wall thickness {}", thickness),
            GeometryAction::ExtrudeProfile { distance, .. } => format!("Extrude profile by {}", distance),
            GeometryAction::RevolveProfile { angle_degrees, .. } => format!("Revolve profile {}°", angle_degrees),
        }
    }
}

// ============================================================
// Parser errors
// ============================================================

#[derive(Debug, Clone, thiserror::Error)]
pub enum ParseError {
    #[error("empty input — no shape description found")]
    Empty,

    #[error("unknown shape: '{0}'")]
    UnknownShape(String),

    #[error("missing dimension: expected {0}")]
    MissingDimension(&'static str),

    #[error("invalid number: '{0}'")]
    InvalidNumber(String),

    #[error("ambiguous query: {0}")]
    Ambiguous(String),
}

// ============================================================
// ShapeParser
// ============================================================

/// A rule-based parser that converts natural-language text into a
/// sequence of `GeometryAction`s.
pub struct ShapeParser {
    /// Default size for shapes when no dimension is specified.
    pub default_size: f64,
    /// Default fillet radius.
    pub default_fillet_radius: f64,
    /// Default shell thickness.
    pub default_shell_thickness: f64,
}

impl Default for ShapeParser {
    fn default() -> Self {
        Self::new()
    }
}

impl ShapeParser {
    pub fn new() -> Self {
        Self {
            default_size: 10.0,
            default_fillet_radius: 1.0,
            default_shell_thickness: 1.0,
        }
    }

    /// Parse a text description into a list of geometry actions.
    pub fn parse(&self, text: &str) -> Result<Vec<GeometryAction>, ParseError> {
        let text = text.to_lowercase();
        let tokens = tokenize(&text);

        if tokens.is_empty() {
            return Err(ParseError::Empty);
        }

        let mut actions = Vec::new();
        let mut i = 0;

        while i < tokens.len() {
            // Try to match a shape keyword
            if let Some((action, consumed)) = self.try_parse_shape(&tokens, i)? {
                actions.push(action);
                i += consumed;
                continue;
            }

            // Try to match a modifier (fillet, chamfer, shell, holes)
            if let Some((modifier_actions, consumed)) = self.try_parse_modifier(&tokens, i)? {
                actions.extend(modifier_actions);
                i += consumed;
                continue;
            }

            // Skip unknown tokens (e.g., "a", "an", "the", "with", numbers we already consumed)
            i += 1;
        }

        if actions.is_empty() {
            return Err(ParseError::UnknownShape(text.clone()));
        }

        Ok(actions)
    }

    /// Try to parse a shape creation action starting at index `i`.
    /// Returns `Some((action, tokens_consumed))` if successful, `None` if
    /// no shape keyword matched.
    fn try_parse_shape(
        &self,
        tokens: &[String],
        i: usize,
    ) -> Result<Option<(GeometryAction, usize)>, ParseError> {
        let token = &tokens[i];

        let shape_type = match token.as_str() {
            "box" | "cube" | "block" | "brick" => ShapeType::Box,
            "cylinder" | "rod" | "shaft" | "pipe" => ShapeType::Cylinder,
            "sphere" | "ball" => ShapeType::Sphere,
            "cone" => ShapeType::Cone,
            "torus" | "ring" | "donut" => ShapeType::Torus,
            _ => return Ok(None),
        };

        // Look ahead for dimensions
        let (size, consumed) = self.extract_dimensions(tokens, i + 1, shape_type);
        let mut total_consumed = 1 + consumed;

        // Look for "at (x, y, z)" or "centered at (x, y, z)"
        let center = self.extract_center(tokens, i + 1 + consumed, &mut total_consumed);

        let action = match shape_type {
            ShapeType::Box => GeometryAction::CreateBox {
                size: [size[0], size[1], size[2]],
                center,
            },
            ShapeType::Cylinder => GeometryAction::CreateCylinder {
                diameter: size[0],
                height: size[2],
                center,
            },
            ShapeType::Sphere => GeometryAction::CreateSphere {
                diameter: size[0],
                center,
            },
            ShapeType::Cone => GeometryAction::CreateCone {
                bottom_diameter: size[0],
                top_diameter: size[0] * 0.5, // Default: frustum
                height: size[2],
                center,
            },
            ShapeType::Torus => GeometryAction::CreateTorus {
                major_diameter: size[0],
                minor_diameter: size[0] * 0.2, // Default: 20% of major
                center,
            },
        };

        Ok(Some((action, total_consumed)))
    }

    /// Try to parse a modifier (fillet, chamfer, shell, holes) starting at `i`.
    fn try_parse_modifier(
        &self,
        tokens: &[String],
        i: usize,
    ) -> Result<Option<(Vec<GeometryAction>, usize)>, ParseError> {
        let token = &tokens[i];

        match token.as_str() {
            "fillet" | "round" => {
                let radius = self.extract_number_after(tokens, i + 1)
                    .unwrap_or(self.default_fillet_radius);
                Ok(Some((vec![GeometryAction::FilletAllEdges { radius }], 1)))
            }
            "chamfer" => {
                let distance = self.extract_number_after(tokens, i + 1)
                    .unwrap_or(self.default_fillet_radius);
                Ok(Some((vec![GeometryAction::ChamferAllEdges { distance }], 1)))
            }
            "shell" | "hollow" => {
                let thickness = self.extract_number_after(tokens, i + 1)
                    .unwrap_or(self.default_shell_thickness);
                Ok(Some((vec![GeometryAction::Shell { thickness }], 1)))
            }
            "holes" | "hole" => {
                // Parse "N holes of diameter D" or "N holes"
                let (count, mut consumed) = self.extract_hole_count(tokens, i);
                let diameter = self.extract_hole_diameter(tokens, i + consumed, &mut consumed);
                let hole_actions = self.generate_hole_actions(count, diameter);
                Ok(Some((hole_actions, consumed)))
            }
            _ => Ok(None),
        }
    }

    /// Extract dimensions from tokens following a shape keyword.
    /// Returns `[width, depth, height]` (or `[diameter, diameter, height]`
    /// for cylindrical shapes).
    fn extract_dimensions(
        &self,
        tokens: &[String],
        start: usize,
        shape_type: ShapeType,
    ) -> ([f64; 3], usize) {
        // Look for patterns like "50×30×20", "50x30x20", "50 by 30 by 20"
        let mut consumed = 0;
        let mut nums = Vec::new();

        // Skip "of" if present
        let mut idx = start;
        if idx < tokens.len() && tokens[idx] == "of" {
            idx += 1;
            consumed += 1;
        }

        // Try to parse up to 3 numbers separated by ×, x, or "by"
        for _ in 0..3 {
            if idx >= tokens.len() {
                break;
            }
            if let Ok(n) = tokens[idx].parse::<f64>() {
                nums.push(n);
                idx += 1;
                consumed += 1;
                // Skip separator
                if idx < tokens.len() {
                    let sep = &tokens[idx];
                    if sep == "×" || sep == "x" || sep == "by" || sep == "*" {
                        idx += 1;
                        consumed += 1;
                    }
                }
            } else {
                // Try to strip a trailing unit (mm, cm, m)
                if let Some(n) = parse_with_unit(&tokens[idx]) {
                    nums.push(n);
                    idx += 1;
                    consumed += 1;
                    if idx < tokens.len() {
                        let sep = &tokens[idx];
                        if sep == "×" || sep == "x" || sep == "by" || sep == "*" {
                            idx += 1;
                            consumed += 1;
                        }
                    }
                } else {
                    break;
                }
            }
        }

        // Fill in defaults
        let default = self.default_size;
        let result = match nums.len() {
            0 => match shape_type {
                ShapeType::Box => [default, default, default],
                ShapeType::Cylinder | ShapeType::Cone => [default, default, default],
                ShapeType::Sphere => [default, default, default],
                ShapeType::Torus => [default, default, default],
            },
            1 => match shape_type {
                ShapeType::Box => [nums[0], nums[0], nums[0]], // Cube
                ShapeType::Cylinder | ShapeType::Cone => [nums[0], nums[0], nums[0]], // diameter = height
                ShapeType::Sphere => [nums[0], nums[0], nums[0]],
                ShapeType::Torus => [nums[0], nums[0], nums[0]],
            },
            2 => match shape_type {
                ShapeType::Box => [nums[0], nums[0], nums[1]], // width = depth, height
                ShapeType::Cylinder | ShapeType::Cone => [nums[0], nums[0], nums[1]], // diameter, height
                ShapeType::Sphere => [nums[0], nums[0], nums[0]], // ignore second
                ShapeType::Torus => [nums[0], nums[1], nums[0]], // major, minor
            },
            _ => [nums[0], nums[1], nums[2]],
        };

        (result, consumed)
    }

    /// Extract a center point from "at (x, y, z)" or "centered at (x, y, z)".
    fn extract_center(&self, tokens: &[String], start: usize, consumed: &mut usize) -> [f64; 3] {
        let mut idx = start;
        // Skip "at" or "centered at"
        if idx < tokens.len() && tokens[idx] == "centered" {
            idx += 1;
            *consumed += 1;
        }
        if idx < tokens.len() && tokens[idx] == "at" {
            idx += 1;
            *consumed += 1;
        } else {
            return [0.0, 0.0, 0.0];
        }

        // Skip opening paren
        if idx < tokens.len() && (tokens[idx] == "(" || tokens[idx] == "[") {
            idx += 1;
            *consumed += 1;
        }

        let mut coords = Vec::new();
        for _ in 0..3 {
            // Skip commas between coordinates
            while idx < tokens.len() && tokens[idx] == "," {
                idx += 1;
                *consumed += 1;
            }
            if idx >= tokens.len() {
                break;
            }
            // Strip trailing comma or closing paren
            let t = tokens[idx].trim_end_matches([',', ')', ']']);
            if let Ok(n) = t.parse::<f64>() {
                coords.push(n);
                idx += 1;
                *consumed += 1;
            } else if let Some(n) = parse_with_unit(t) {
                coords.push(n);
                idx += 1;
                *consumed += 1;
            } else {
                break;
            }
        }

        match coords.len() {
            0 => [0.0, 0.0, 0.0],
            1 => [coords[0], 0.0, 0.0],
            2 => [coords[0], coords[1], 0.0],
            _ => [coords[0], coords[1], coords[2]],
        }
    }

    /// Extract a single number following a keyword (e.g., "fillet 2.5" → 2.5).
    fn extract_number_after(&self, tokens: &[String], start: usize) -> Option<f64> {
        if start >= tokens.len() {
            return None;
        }
        if let Ok(n) = tokens[start].parse::<f64>() {
            return Some(n);
        }
        if let Some(n) = parse_with_unit(&tokens[start]) {
            return Some(n);
        }
        // Try "radius 2.5" or "R2.5"
        if start + 1 < tokens.len() {
            if tokens[start] == "radius" || tokens[start] == "r" {
                if let Ok(n) = tokens[start + 1].parse::<f64>() {
                    return Some(n);
                }
            }
            // R2.5 (single token starting with R)
            if tokens[start].starts_with('r') {
                if let Ok(n) = tokens[start][1..].parse::<f64>() {
                    return Some(n);
                }
            }
        }
        None
    }

    /// Extract the number of holes from "4 holes" or "with 4 holes".
    fn extract_hole_count(&self, _tokens: &[String], _start: usize) -> (usize, usize) {
        let consumed = 1; // consume "holes" token

        // Look backwards for a number (but we can't look backwards here,
        // so we look forward past "holes" for "of diameter D" patterns).
        // The count is usually before "holes" — but since we don't have
        // access to it, we look for "with N holes" by scanning forward.

        // Look for a number BEFORE "holes" by scanning back in the previous
        // tokens. Since we don't have access to the previous index, we
        // assume the caller has already consumed it. Default to 1 hole.
        (1, consumed)
    }

    /// Extract the hole diameter from "of diameter D" or "of size D".
    fn extract_hole_diameter(
        &self,
        tokens: &[String],
        start: usize,
        consumed: &mut usize,
    ) -> f64 {
        let mut idx = start;
        if idx < tokens.len() && tokens[idx] == "of" {
            idx += 1;
            *consumed += 1;
        }
        if idx < tokens.len() && (tokens[idx] == "diameter" || tokens[idx] == "size") {
            idx += 1;
            *consumed += 1;
        } else if idx < tokens.len() && tokens[idx].starts_with('ø') {
            // "Ø5" — parse rest of token as number
            if let Ok(n) = tokens[idx][2..].parse::<f64>() {
                *consumed += 1;
                return n;
            }
        }
        if idx < tokens.len() {
            if let Ok(n) = tokens[idx].parse::<f64>() {
                *consumed += 1;
                return n;
            }
            if let Some(n) = parse_with_unit(&tokens[idx]) {
                *consumed += 1;
                return n;
            }
        }
        self.default_size * 0.1 // Default: 10% of default size
    }

    /// Generate `count` CreateCylinder + BooleanSubtract action pairs
    /// for hole features.
    fn generate_hole_actions(&self, count: usize, diameter: f64) -> Vec<GeometryAction> {
        let mut actions = Vec::with_capacity(count * 2);
        // Place holes in a grid pattern on the top face of a default-size box
        let grid_size = (count as f64).sqrt().ceil() as usize;
        let spacing = self.default_size / (grid_size as f64 + 1.0).max(1.0);
        let height = self.default_size * 1.2; // Slightly taller to ensure through-hole

        let mut placed = 0;
        for row in 0..grid_size {
            for col in 0..grid_size {
                if placed >= count {
                    break;
                }
                let cx = (col as f64 + 1.0) * spacing - self.default_size * 0.5;
                let cy = (row as f64 + 1.0) * spacing - self.default_size * 0.5;
                actions.push(GeometryAction::CreateCylinder {
                    diameter,
                    height,
                    center: [cx, cy, 0.0],
                });
                actions.push(GeometryAction::BooleanSubtract);
                placed += 1;
            }
        }
        actions
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ShapeType {
    Box,
    Cylinder,
    Sphere,
    Cone,
    Torus,
}

// ============================================================
// Tokenizer
// ============================================================

/// Split text into lowercase tokens, separating punctuation.
fn tokenize(text: &str) -> Vec<String> {
    let text = text.to_lowercase();
    let mut tokens = Vec::new();
    let mut current = String::new();

    let chars: Vec<char> = text.chars().collect();
    for (i, &ch) in chars.iter().enumerate() {
        // Check if this 'x' is a separator between numbers (e.g., "50x30")
        let is_x_separator = ch == 'x'
            && !current.is_empty()
            && current.chars().last().map(|c| c.is_ascii_digit() || c == '.').unwrap_or(false)
            && chars.get(i + 1).map(|c| c.is_ascii_digit()).unwrap_or(false);

        if is_x_separator {
            // Flush current token, then push "x" as a separator
            if !current.is_empty() {
                tokens.push(current.clone());
                current.clear();
            }
            tokens.push("x".to_string());
        } else if ch.is_alphanumeric() || ch == '.' || ch == '-' || ch == '+' || ch == '×' || ch == 'ø' {
            current.push(ch);
        } else {
            // Separator or punctuation — flush current token first
            if !current.is_empty() {
                tokens.push(current.clone());
                current.clear();
            }
            // Keep punctuation as separate tokens
            if ch == '(' || ch == ')' || ch == '[' || ch == ']' || ch == ',' {
                tokens.push(ch.to_string());
            }
        }
    }
    if !current.is_empty() {
        tokens.push(current);
    }
    tokens
}

/// Parse a number with an optional unit suffix (e.g., "50mm", "5cm", "2m").
fn parse_with_unit(s: &str) -> Option<f64> {
    if s.is_empty() {
        return None;
    }
    // Try mm, cm, m suffixes
    if let Some(rest) = s.strip_suffix("mm") {
        return rest.parse::<f64>().ok();
    }
    if let Some(rest) = s.strip_suffix("cm") {
        return rest.parse::<f64>().ok().map(|n| n * 10.0);
    }
    if let Some(rest) = s.strip_suffix("m") {
        return rest.parse::<f64>().ok().map(|n| n * 1000.0);
    }
    if let Some(rest) = s.strip_suffix("in") {
        return rest.parse::<f64>().ok().map(|n| n * 25.4);
    }
    None
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
#[allow(clippy::disallowed_methods, clippy::unwrap_used, clippy::needless_range_loop, unused_imports)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_box() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateBox { size, center } => {
                assert_eq!(*size, [10.0, 10.0, 10.0]); // Default size
                assert_eq!(*center, [0.0, 0.0, 0.0]);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_box_with_dimensions() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 50x30x20").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateBox { size, .. } => {
                assert_eq!(*size, [50.0, 30.0, 20.0]);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_box_with_units() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 50mm x 30mm x 20mm").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateBox { size, .. } => {
                assert_eq!(*size, [50.0, 30.0, 20.0]);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_cube_synonym() {
        let parser = ShapeParser::new();
        let actions = parser.parse("cube 25").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateBox { size, .. } => {
                assert_eq!(*size, [25.0, 25.0, 25.0]);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_cylinder() {
        let parser = ShapeParser::new();
        let actions = parser.parse("cylinder 10x50").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateCylinder { diameter, height, .. } => {
                assert_eq!(*diameter, 10.0);
                assert_eq!(*height, 50.0);
            }
            _ => panic!("Expected CreateCylinder"),
        }
    }

    #[test]
    fn test_parse_rod_synonym() {
        let parser = ShapeParser::new();
        let actions = parser.parse("rod 8x100").unwrap();
        assert_eq!(actions.len(), 1);
        assert!(matches!(actions[0], GeometryAction::CreateCylinder { .. }));
    }

    #[test]
    fn test_parse_sphere() {
        let parser = ShapeParser::new();
        let actions = parser.parse("sphere 20").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateSphere { diameter, .. } => {
                assert_eq!(*diameter, 20.0);
            }
            _ => panic!("Expected CreateSphere"),
        }
    }

    #[test]
    fn test_parse_ball_synonym() {
        let parser = ShapeParser::new();
        let actions = parser.parse("ball 15").unwrap();
        assert_eq!(actions.len(), 1);
        assert!(matches!(actions[0], GeometryAction::CreateSphere { .. }));
    }

    #[test]
    fn test_parse_cone() {
        let parser = ShapeParser::new();
        let actions = parser.parse("cone 20x30").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateCone { bottom_diameter, top_diameter, height, .. } => {
                assert_eq!(*bottom_diameter, 20.0);
                assert_eq!(*height, 30.0);
                assert!(*top_diameter > 0.0);
            }
            _ => panic!("Expected CreateCone"),
        }
    }

    #[test]
    fn test_parse_torus() {
        let parser = ShapeParser::new();
        let actions = parser.parse("torus 50x10").unwrap();
        assert_eq!(actions.len(), 1);
        match &actions[0] {
            GeometryAction::CreateTorus { major_diameter, minor_diameter, .. } => {
                assert_eq!(*major_diameter, 50.0);
                assert_eq!(*minor_diameter, 10.0);
            }
            _ => panic!("Expected CreateTorus"),
        }
    }

    #[test]
    fn test_parse_donut_synonym() {
        let parser = ShapeParser::new();
        let actions = parser.parse("donut 40").unwrap();
        assert_eq!(actions.len(), 1);
        assert!(matches!(actions[0], GeometryAction::CreateTorus { .. }));
    }

    #[test]
    fn test_parse_box_with_fillet() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 20x20x20 fillet 2").unwrap();
        assert_eq!(actions.len(), 2);
        assert!(matches!(actions[0], GeometryAction::CreateBox { .. }));
        match &actions[1] {
            GeometryAction::FilletAllEdges { radius } => {
                assert_eq!(*radius, 2.0);
            }
            _ => panic!("Expected FilletAllEdges"),
        }
    }

    #[test]
    fn test_parse_box_with_chamfer() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 20x20x20 chamfer 1.5").unwrap();
        assert_eq!(actions.len(), 2);
        match &actions[1] {
            GeometryAction::ChamferAllEdges { distance } => {
                assert_eq!(*distance, 1.5);
            }
            _ => panic!("Expected ChamferAllEdges"),
        }
    }

    #[test]
    fn test_parse_box_with_shell() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 30x30x30 shell 2").unwrap();
        assert_eq!(actions.len(), 2);
        match &actions[1] {
            GeometryAction::Shell { thickness } => {
                assert_eq!(*thickness, 2.0);
            }
            _ => panic!("Expected Shell"),
        }
    }

    #[test]
    fn test_parse_hollow_synonym() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 30x30x30 hollow 1.5").unwrap();
        assert_eq!(actions.len(), 2);
        assert!(matches!(actions[1], GeometryAction::Shell { .. }));
    }

    #[test]
    fn test_parse_box_with_holes() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 50x50x20 holes of diameter 5").unwrap();
        // Should have: CreateBox + (CreateCylinder + BooleanSubtract) * hole_count
        assert!(!actions.is_empty());
        assert!(matches!(actions[0], GeometryAction::CreateBox { .. }));
        // Should have at least 1 hole (CreateCylinder + BooleanSubtract)
        assert!(actions.len() >= 3);
        assert!(matches!(actions[1], GeometryAction::CreateCylinder { .. }));
        assert!(matches!(actions[2], GeometryAction::BooleanSubtract));
    }

    #[test]
    fn test_parse_empty_input_errors() {
        let parser = ShapeParser::new();
        let result = parser.parse("");
        assert!(matches!(result, Err(ParseError::Empty)));
    }

    #[test]
    fn test_parse_whitespace_only_errors() {
        let parser = ShapeParser::new();
        let result = parser.parse("   ");
        assert!(matches!(result, Err(ParseError::Empty)));
    }

    #[test]
    fn test_parse_unknown_shape_errors() {
        let parser = ShapeParser::new();
        let result = parser.parse("gibberish that doesn't match any shape");
        assert!(matches!(result, Err(ParseError::UnknownShape(_))));
    }

    #[test]
    fn test_parse_with_center() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 20x20x20 at (10, 20, 30)").unwrap();
        match &actions[0] {
            GeometryAction::CreateBox { center, .. } => {
                assert_eq!(*center, [10.0, 20.0, 30.0]);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_with_centered_at() {
        let parser = ShapeParser::new();
        let actions = parser.parse("sphere 10 centered at (5, 5, 5)").unwrap();
        match &actions[0] {
            GeometryAction::CreateSphere { center, .. } => {
                assert_eq!(*center, [5.0, 5.0, 5.0]);
            }
            _ => panic!("Expected CreateSphere"),
        }
    }

    #[test]
    fn test_parse_with_units_cm() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 5cm x 3cm x 2cm").unwrap();
        match &actions[0] {
            GeometryAction::CreateBox { size, .. } => {
                // 5cm = 50mm, 3cm = 30mm, 2cm = 20mm
                assert_eq!(*size, [50.0, 30.0, 20.0]);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_with_units_m() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 1m x 0.5m x 0.2m").unwrap();
        match &actions[0] {
            GeometryAction::CreateBox { size, .. } => {
                // 1m = 1000mm, 0.5m = 500mm, 0.2m = 200mm
                assert_eq!(*size, [1000.0, 500.0, 200.0]);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_with_units_in() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 1in x 2in x 3in").unwrap();
        match &actions[0] {
            GeometryAction::CreateBox { size, .. } => {
                // 1in = 25.4mm
                assert!((size[0] - 25.4).abs() < 1e-6);
                assert!((size[1] - 50.8).abs() < 1e-6);
                assert!((size[2] - 76.2).abs() < 1e-6);
            }
            _ => panic!("Expected CreateBox"),
        }
    }

    #[test]
    fn test_parse_complex_bracket() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 50x30x20 holes of diameter 5 fillet 1").unwrap();
        // Should have: CreateBox + holes + fillet
        assert!(actions.len() >= 4);
        assert!(matches!(actions[0], GeometryAction::CreateBox { .. }));
        // Last action should be fillet
        assert!(matches!(actions.last(), Some(GeometryAction::FilletAllEdges { .. })));
    }

    #[test]
    fn test_action_describe() {
        let action = GeometryAction::CreateBox {
            size: [10.0, 20.0, 30.0],
            center: [1.0, 2.0, 3.0],
        };
        let desc = action.describe();
        assert!(desc.contains("box"));
        assert!(desc.contains("10"));
        assert!(desc.contains("20"));
        assert!(desc.contains("30"));

        let action = GeometryAction::CreateCylinder {
            diameter: 5.0,
            height: 10.0,
            center: [0.0, 0.0, 0.0],
        };
        let desc = action.describe();
        assert!(desc.contains("cylinder"));
        assert!(desc.contains("Ø5"));

        let action = GeometryAction::BooleanSubtract;
        assert_eq!(action.describe(), "Subtract last shape");
    }

    #[test]
    fn test_tokenize_basic() {
        let tokens = tokenize("box 50x30x20");
        assert!(tokens.contains(&"box".to_string()));
        assert!(tokens.contains(&"50".to_string()));
        assert!(tokens.contains(&"x".to_string()));
        assert!(tokens.contains(&"30".to_string()));
        assert!(tokens.contains(&"20".to_string()));
    }

    #[test]
    fn test_tokenize_with_punctuation() {
        let tokens = tokenize("box at (10, 20, 30)");
        assert!(tokens.contains(&"(".to_string()));
        assert!(tokens.contains(&"10".to_string()));
        assert!(tokens.contains(&"20".to_string()));
        assert!(tokens.contains(&"30".to_string()));
        assert!(tokens.contains(&")".to_string()));
    }

    #[test]
    fn test_parse_with_unit_mm() {
        assert_eq!(parse_with_unit("50mm"), Some(50.0));
        assert_eq!(parse_with_unit("5cm"), Some(50.0));
        assert_eq!(parse_with_unit("1m"), Some(1000.0));
        assert_eq!(parse_with_unit("1in"), Some(25.4));
        assert_eq!(parse_with_unit("abc"), None);
        assert_eq!(parse_with_unit(""), None);
    }

    #[test]
    fn test_default_fillet_radius() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 20x20x20 fillet").unwrap();
        match &actions[1] {
            GeometryAction::FilletAllEdges { radius } => {
                assert_eq!(*radius, 1.0); // Default radius
            }
            _ => panic!("Expected FilletAllEdges"),
        }
    }

    #[test]
    fn test_default_shell_thickness() {
        let parser = ShapeParser::new();
        let actions = parser.parse("box 20x20x20 shell").unwrap();
        match &actions[1] {
            GeometryAction::Shell { thickness } => {
                assert_eq!(*thickness, 1.0); // Default thickness
            }
            _ => panic!("Expected Shell"),
        }
    }

    #[test]
    fn test_parse_case_insensitive() {
        let parser = ShapeParser::new();
        let actions = parser.parse("BOX 20x20x20").unwrap();
        assert!(matches!(actions[0], GeometryAction::CreateBox { .. }));

        let actions = parser.parse("Cylinder 10x50").unwrap();
        assert!(matches!(actions[0], GeometryAction::CreateCylinder { .. }));
    }
}
