// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! Shape from Text — natural language to geometry parser.
//!
//! Per BREPCAD_IMPLEMENTATION_PLAN.md Phase 3.3: "Shape from Text" feature
//! that parses text descriptions and creates primitive shapes.
//!
//! Per motivated edit: uses a lightweight rule-based parser (not LLM) for
//! WASM compatibility. The parser recognizes keywords for shape types
//! and extracts dimensions from the text.
//!
//! # Supported Keywords
//!
//! - **Box/Cube/Block**: creates a box with width×height×depth
//! - **Cylinder/Rod/Pin**: creates a cylinder with radius×height
//! - **Sphere/Ball**: creates a sphere with radius
//! - **Cone**: creates a cone with radius×height
//! - **Torus/Ring**: creates a torus with major×minor radius
//! - **Tube/Pipe**: creates a hollow cylinder (extrude + subtract)
//! - **Plate/Sheet**: creates a thin flat box
//! - **Bracket/L-bracket**: creates an L-shaped bracket (2 boxes)
//!
//! # Dimension Extraction
//!
//! Dimensions can be specified as:
//! - "50mm", "50 mm", "50" — millimeters
//! - "width 100", "w=100", "100 wide" — keyword + value
//! - "100x50x25" — WxHxD format
//! - "radius 10", "r=10", "diameter 20" — radius/diameter keywords

use std::collections::HashMap;

// ============================================================
// Error types
// ============================================================

#[derive(Debug, Clone, thiserror::Error)]
pub enum ShapeFromTextError {
    #[error("No shape keyword found in text: '{0}'")]
    NoShapeKeyword(String),

    #[error("Missing required dimension '{0}' for shape '{1}'")]
    MissingDimension(String, String),

    #[error("Invalid dimension value: '{0}'")]
    InvalidDimension(String),
}

// ============================================================
// Shape Description
// ============================================================

/// A parsed shape description from text.
#[derive(Debug, Clone, PartialEq)]
pub enum ShapeDescription {
    /// Box: width × height × depth.
    Box { width: f64, height: f64, depth: f64 },
    /// Cylinder: radius × height.
    Cylinder { radius: f64, height: f64 },
    /// Sphere: radius.
    Sphere { radius: f64 },
    /// Cone: bottom_radius × height.
    Cone { radius: f64, height: f64 },
    /// Torus: major_radius × minor_radius.
    Torus { major_radius: f64, minor_radius: f64 },
    /// Tube: outer_radius × inner_radius × height.
    Tube { outer_radius: f64, inner_radius: f64, height: f64 },
    /// Plate: thin flat box (width × height × thickness).
    Plate { width: f64, height: f64, thickness: f64 },
    /// L-bracket: two flanges.
    LBracket { flange1_length: f64, flange2_length: f64, width: f64, thickness: f64 },
}

impl ShapeDescription {
    /// Get the human-readable name of the shape.
    pub fn shape_name(&self) -> &'static str {
        match self {
            ShapeDescription::Box { .. } => "Box",
            ShapeDescription::Cylinder { .. } => "Cylinder",
            ShapeDescription::Sphere { .. } => "Sphere",
            ShapeDescription::Cone { .. } => "Cone",
            ShapeDescription::Torus { .. } => "Torus",
            ShapeDescription::Tube { .. } => "Tube",
            ShapeDescription::Plate { .. } => "Plate",
            ShapeDescription::LBracket { .. } => "L-Bracket",
        }
    }

    /// Get all dimensions as a HashMap (name → value).
    pub fn dimensions(&self) -> HashMap<String, f64> {
        let mut dims = HashMap::new();
        match self {
            ShapeDescription::Box { width, height, depth } => {
                dims.insert("width".to_string(), *width);
                dims.insert("height".to_string(), *height);
                dims.insert("depth".to_string(), *depth);
            }
            ShapeDescription::Cylinder { radius, height } => {
                dims.insert("radius".to_string(), *radius);
                dims.insert("height".to_string(), *height);
            }
            ShapeDescription::Sphere { radius } => {
                dims.insert("radius".to_string(), *radius);
            }
            ShapeDescription::Cone { radius, height } => {
                dims.insert("radius".to_string(), *radius);
                dims.insert("height".to_string(), *height);
            }
            ShapeDescription::Torus { major_radius, minor_radius } => {
                dims.insert("major_radius".to_string(), *major_radius);
                dims.insert("minor_radius".to_string(), *minor_radius);
            }
            ShapeDescription::Tube { outer_radius, inner_radius, height } => {
                dims.insert("outer_radius".to_string(), *outer_radius);
                dims.insert("inner_radius".to_string(), *inner_radius);
                dims.insert("height".to_string(), *height);
            }
            ShapeDescription::Plate { width, height, thickness } => {
                dims.insert("width".to_string(), *width);
                dims.insert("height".to_string(), *height);
                dims.insert("thickness".to_string(), *thickness);
            }
            ShapeDescription::LBracket { flange1_length, flange2_length, width, thickness } => {
                dims.insert("flange1_length".to_string(), *flange1_length);
                dims.insert("flange2_length".to_string(), *flange2_length);
                dims.insert("width".to_string(), *width);
                dims.insert("thickness".to_string(), *thickness);
            }
        }
        dims
    }
}

// ============================================================
// Text Parser
// ============================================================

/// Parse a text description into a ShapeDescription.
///
/// Per BREPCAD Phase 3.3: rule-based parser that recognizes shape
/// keywords and extracts dimensions. No external ML dependency.
pub struct ShapeParser;

impl ShapeParser {
    /// Parse text into a shape description.
    pub fn parse(text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let lower = text.to_lowercase();

        // Detect shape type
        let shape_type = Self::detect_shape_type(&lower)?;

        // Extract all dimensions from the text
        let dims = Self::extract_dimensions(&lower);

        // Build the shape description based on type
        match shape_type {
            "box" | "cube" | "block" => Self::parse_box(&dims, text),
            "cylinder" | "rod" | "pin" => Self::parse_cylinder(&dims, text),
            "sphere" | "ball" => Self::parse_sphere(&dims, text),
            "cone" => Self::parse_cone(&dims, text),
            "torus" | "ring" => Self::parse_torus(&dims, text),
            "tube" | "pipe" => Self::parse_tube(&dims, text),
            "plate" | "sheet" => Self::parse_plate(&dims, text),
            "bracket" | "l-bracket" | "lbracket" => Self::parse_l_bracket(&dims, text),
            _ => Err(ShapeFromTextError::NoShapeKeyword(text.to_string())),
        }
    }

    /// Detect the shape type from keywords.
    fn detect_shape_type(lower: &str) -> Result<&'static str, ShapeFromTextError> {
        // Check for tube/pipe before cylinder (tube contains cylinder-like keywords)
        if lower.contains("tube") || lower.contains("pipe") {
            return Ok("tube");
        }
        if lower.contains("bracket") || lower.contains("l-bracket") || lower.contains("lbracket") {
            return Ok("bracket");
        }
        if lower.contains("plate") || lower.contains("sheet") {
            return Ok("plate");
        }
        if lower.contains("box") || lower.contains("cube") || lower.contains("block") {
            return Ok("box");
        }
        if lower.contains("cylinder") || lower.contains("rod") || lower.contains("pin") {
            return Ok("cylinder");
        }
        if lower.contains("sphere") || lower.contains("ball") {
            return Ok("sphere");
        }
        if lower.contains("cone") {
            return Ok("cone");
        }
        if lower.contains("torus") || lower.contains("ring") {
            return Ok("torus");
        }
        Err(ShapeFromTextError::NoShapeKeyword(lower.to_string()))
    }

    /// Extract all dimensions from text using word-boundary matching.
    fn extract_dimensions(lower: &str) -> HashMap<String, f64> {
        let mut dims = HashMap::new();

        // Tokenize: split into words, preserving "key=value" pairs
        let tokens = Self::tokenize(lower);

        // Extract WxHxD format from tokens
        Self::extract_whd_from_tokens(&tokens, &mut dims);

        // Extract keyword-based dimensions using word matching
        Self::extract_keyword_from_tokens(&tokens, &["width", "w"], "width", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["height", "h"], "height", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["depth", "d", "length", "len", "l"], "depth", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["radius", "r"], "radius", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["diameter", "dia"], "diameter", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["major"], "major_radius", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["minor"], "minor_radius", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["thickness", "thk", "t"], "thickness", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["outer", "od"], "outer_radius", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["inner", "id"], "inner_radius", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["flange1"], "flange1_length", &mut dims);
        Self::extract_keyword_from_tokens(&tokens, &["flange2"], "flange2_length", &mut dims);

        // Convert diameter to radius if radius not specified
        if let Some(&dia) = dims.get("diameter") {
            if !dims.contains_key("radius") {
                dims.insert("radius".to_string(), dia / 2.0);
            }
            if !dims.contains_key("outer_radius") {
                dims.insert("outer_radius".to_string(), dia / 2.0);
            }
        }

        // Extract bare numbers (fallback) — only fill in missing dimensions
        let bare_nums = Self::extract_bare_numbers_from_tokens(&tokens);
        if bare_nums.len() == 1 && dims.is_empty() {
            dims.insert("radius".to_string(), bare_nums[0]);
        } else if bare_nums.len() == 2 && dims.is_empty() {
            dims.insert("radius".to_string(), bare_nums[0]);
            dims.insert("height".to_string(), bare_nums[1]);
        } else if bare_nums.len() >= 3 {
            // Only fill dimensions that aren't already set by keywords
            if !dims.contains_key("width") {
                dims.insert("width".to_string(), bare_nums[0]);
            }
            if !dims.contains_key("height") {
                dims.insert("height".to_string(), bare_nums[1]);
            }
            if !dims.contains_key("depth") {
                dims.insert("depth".to_string(), bare_nums[2]);
            }
        }

        dims
    }

    /// Tokenize text: split on whitespace, separate "key=value" into ["key", "value"].
    fn tokenize(lower: &str) -> Vec<String> {
        let mut tokens = Vec::new();
        for word in lower.split_whitespace() {
            // Handle "key=value" → ["key", "value"]
            if let Some(eq_pos) = word.find('=') {
                if eq_pos > 0 && eq_pos < word.len() - 1 {
                    tokens.push(word[..eq_pos].to_string());
                    tokens.push(word[eq_pos + 1..].to_string());
                    continue;
                }
            }
            // Handle "key:value" → ["key", "value"]
            if let Some(colon_pos) = word.find(':') {
                if colon_pos > 0 && colon_pos < word.len() - 1 {
                    tokens.push(word[..colon_pos].to_string());
                    tokens.push(word[colon_pos + 1..].to_string());
                    continue;
                }
            }
            tokens.push(word.to_string());
        }
        tokens
    }

    /// Extract WxHxD format from tokens (e.g., "100x50x25" → width=100, height=50, depth=25).
    fn extract_whd_from_tokens(tokens: &[String], dims: &mut HashMap<String, f64>) {
        for token in tokens {
            // Check if token contains 'x' separating numbers
            let parts: Vec<&str> = token.split('x').collect();
            if parts.len() == 3 {
                if let (Some(w), Some(h), Some(d)) = (
                    Self::parse_number(parts[0]),
                    Self::parse_number(parts[1]),
                    Self::parse_number(parts[2]),
                ) {
                    dims.insert("width".to_string(), w);
                    dims.insert("height".to_string(), h);
                    dims.insert("depth".to_string(), d);
                    return;
                }
            }
            // Handle 2-part WxH
            if parts.len() == 2 {
                if let (Some(w), Some(h)) = (
                    Self::parse_number(parts[0]),
                    Self::parse_number(parts[1]),
                ) {
                    if !dims.contains_key("width") {
                        dims.insert("width".to_string(), w);
                    }
                    if !dims.contains_key("height") {
                        dims.insert("height".to_string(), h);
                    }
                }
            }
        }
    }

    /// Extract keyword-based dimension using word-boundary matching.
    fn extract_keyword_from_tokens(tokens: &[String], keywords: &[&str], dim_name: &str, dims: &mut HashMap<String, f64>) {
        for (i, token) in tokens.iter().enumerate() {
            // Check if this token matches a keyword (exact word match)
            if keywords.contains(&token.as_str()) {
                // Next token should be the value
                if i + 1 < tokens.len() {
                    if let Some(num) = Self::parse_number(&tokens[i + 1]) {
                        dims.insert(dim_name.to_string(), num);
                        return;
                    }
                }
            }
        }
    }

    /// Extract bare numbers from tokens.
    fn extract_bare_numbers_from_tokens(tokens: &[String]) -> Vec<f64> {
        let mut nums = Vec::new();
        for token in tokens {
            if let Some(num) = Self::parse_number(token) {
                nums.push(num);
            }
        }
        nums
    }

    /// Parse a number from a string that may contain units (mm, cm, m).
    fn parse_number(s: &str) -> Option<f64> {
        let s = s.trim();
        let s = s.trim_end_matches("mm").trim_end_matches("cm").trim_end_matches("m");
        let s = s.trim_end_matches(',');
        let s = s.trim();
        s.parse::<f64>().ok()
    }

    // ---------- Shape-specific parsers ----------

    fn parse_box(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let width = dims.get("width").copied().unwrap_or(50.0);
        let height = dims.get("height").copied().unwrap_or(50.0);
        let depth = dims.get("depth").copied().unwrap_or(50.0);

        Self::validate_positive(width, "width", text)?;
        Self::validate_positive(height, "height", text)?;
        Self::validate_positive(depth, "depth", text)?;

        Ok(ShapeDescription::Box { width, height, depth })
    }

    fn parse_cylinder(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let radius = dims.get("radius").copied().unwrap_or(10.0);
        let height = dims.get("height").or(dims.get("depth")).copied().unwrap_or(50.0);

        Self::validate_positive(radius, "radius", text)?;
        Self::validate_positive(height, "height", text)?;

        Ok(ShapeDescription::Cylinder { radius, height })
    }

    fn parse_sphere(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let radius = dims.get("radius").copied().unwrap_or(25.0);
        Self::validate_positive(radius, "radius", text)?;
        Ok(ShapeDescription::Sphere { radius })
    }

    fn parse_cone(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let radius = dims.get("radius").copied().unwrap_or(15.0);
        let height = dims.get("height").or(dims.get("depth")).copied().unwrap_or(40.0);

        Self::validate_positive(radius, "radius", text)?;
        Self::validate_positive(height, "height", text)?;

        Ok(ShapeDescription::Cone { radius, height })
    }

    fn parse_torus(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let major = dims.get("major_radius").copied().unwrap_or(20.0);
        let minor = dims.get("minor_radius").copied().unwrap_or(5.0);

        Self::validate_positive(major, "major_radius", text)?;
        Self::validate_positive(minor, "minor_radius", text)?;

        Ok(ShapeDescription::Torus { major_radius: major, minor_radius: minor })
    }

    fn parse_tube(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let outer = dims.get("outer_radius").or(dims.get("radius")).copied().unwrap_or(15.0);
        let inner = dims.get("inner_radius").copied().unwrap_or(10.0);
        let height = dims.get("height").or(dims.get("depth")).copied().unwrap_or(50.0);

        Self::validate_positive(outer, "outer_radius", text)?;
        Self::validate_positive(inner, "inner_radius", text)?;
        Self::validate_positive(height, "height", text)?;

        if inner >= outer {
            return Err(ShapeFromTextError::InvalidDimension(
                format!("inner_radius ({}) must be < outer_radius ({})", inner, outer)
            ));
        }

        Ok(ShapeDescription::Tube { outer_radius: outer, inner_radius: inner, height })
    }

    fn parse_plate(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let width = dims.get("width").copied().unwrap_or(100.0);
        let height = dims.get("height").copied().unwrap_or(100.0);
        let thickness = dims.get("thickness").copied().unwrap_or(2.0);

        Self::validate_positive(width, "width", text)?;
        Self::validate_positive(height, "height", text)?;
        Self::validate_positive(thickness, "thickness", text)?;

        Ok(ShapeDescription::Plate { width, height, thickness })
    }

    fn parse_l_bracket(dims: &HashMap<String, f64>, text: &str) -> Result<ShapeDescription, ShapeFromTextError> {
        let flange1 = dims.get("flange1_length").or(dims.get("depth")).copied().unwrap_or(50.0);
        let flange2 = dims.get("flange2_length").copied().unwrap_or(30.0);
        let width = dims.get("width").copied().unwrap_or(40.0);
        let thickness = dims.get("thickness").copied().unwrap_or(3.0);

        Self::validate_positive(flange1, "flange1_length", text)?;
        Self::validate_positive(flange2, "flange2_length", text)?;
        Self::validate_positive(width, "width", text)?;
        Self::validate_positive(thickness, "thickness", text)?;

        Ok(ShapeDescription::LBracket {
            flange1_length: flange1,
            flange2_length: flange2,
            width,
            thickness,
        })
    }

    fn validate_positive(value: f64, name: &str, text: &str) -> Result<(), ShapeFromTextError> {
        if value <= 0.0 {
            return Err(ShapeFromTextError::InvalidDimension(
                format!("{} must be positive (got {}) in text: '{}'", name, value, text)
            ));
        }
        Ok(())
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_box_basic() {
        let shape = ShapeParser::parse("create a box 100x50x25mm").unwrap();
        match shape {
            ShapeDescription::Box { width, height, depth } => {
                assert!((width - 100.0).abs() < 1e-6);
                assert!((height - 50.0).abs() < 1e-6);
                assert!((depth - 25.0).abs() < 1e-6);
            }
            _ => panic!("Expected Box"),
        }
    }

    #[test]
    fn test_parse_box_keywords() {
        let shape = ShapeParser::parse("box width=100 height=50 depth=25").unwrap();
        assert_eq!(shape.shape_name(), "Box");
        let dims = shape.dimensions();
        assert!((dims["width"] - 100.0).abs() < 1e-6);
        assert!((dims["height"] - 50.0).abs() < 1e-6);
        assert!((dims["depth"] - 25.0).abs() < 1e-6);
    }

    #[test]
    fn test_parse_box_defaults() {
        let shape = ShapeParser::parse("create a cube").unwrap();
        match shape {
            ShapeDescription::Box { width, height, depth } => {
                assert!((width - 50.0).abs() < 1e-6);
                assert!((height - 50.0).abs() < 1e-6);
                assert!((depth - 50.0).abs() < 1e-6);
            }
            _ => panic!("Expected Box"),
        }
    }

    #[test]
    fn test_parse_cylinder() {
        let shape = ShapeParser::parse("cylinder radius 10mm height 50mm").unwrap();
        match shape {
            ShapeDescription::Cylinder { radius, height } => {
                assert!((radius - 10.0).abs() < 1e-6);
                assert!((height - 50.0).abs() < 1e-6);
            }
            _ => panic!("Expected Cylinder"),
        }
    }

    #[test]
    fn test_parse_cylinder_diameter() {
        let shape = ShapeParser::parse("cylinder diameter 20 height 100").unwrap();
        match shape {
            ShapeDescription::Cylinder { radius, height } => {
                assert!((radius - 10.0).abs() < 1e-6); // diameter 20 → radius 10
                assert!((height - 100.0).abs() < 1e-6);
            }
            _ => panic!("Expected Cylinder"),
        }
    }

    #[test]
    fn test_parse_sphere() {
        let shape = ShapeParser::parse("sphere radius 25mm").unwrap();
        match shape {
            ShapeDescription::Sphere { radius } => {
                assert!((radius - 25.0).abs() < 1e-6);
            }
            _ => panic!("Expected Sphere"),
        }
    }

    #[test]
    fn test_parse_cone() {
        let shape = ShapeParser::parse("cone r=15 h=40").unwrap();
        match shape {
            ShapeDescription::Cone { radius, height } => {
                assert!((radius - 15.0).abs() < 1e-6);
                assert!((height - 40.0).abs() < 1e-6);
            }
            _ => panic!("Expected Cone"),
        }
    }

    #[test]
    fn test_parse_torus() {
        let shape = ShapeParser::parse("torus major 20 minor 5").unwrap();
        match shape {
            ShapeDescription::Torus { major_radius, minor_radius } => {
                assert!((major_radius - 20.0).abs() < 1e-6);
                assert!((minor_radius - 5.0).abs() < 1e-6);
            }
            _ => panic!("Expected Torus"),
        }
    }

    #[test]
    fn test_parse_tube() {
        let shape = ShapeParser::parse("tube outer 15 inner 10 height 50").unwrap();
        match shape {
            ShapeDescription::Tube { outer_radius, inner_radius, height } => {
                assert!((outer_radius - 15.0).abs() < 1e-6);
                assert!((inner_radius - 10.0).abs() < 1e-6);
                assert!((height - 50.0).abs() < 1e-6);
            }
            _ => panic!("Expected Tube"),
        }
    }

    #[test]
    fn test_parse_tube_inner_greater_than_outer_fails() {
        let result = ShapeParser::parse("tube outer 10 inner 15 height 50");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_plate() {
        let shape = ShapeParser::parse("plate 100x100 thickness 2mm").unwrap();
        match shape {
            ShapeDescription::Plate { width, height, thickness } => {
                assert!((width - 100.0).abs() < 1e-6);
                assert!((height - 100.0).abs() < 1e-6);
                assert!((thickness - 2.0).abs() < 1e-6);
            }
            _ => panic!("Expected Plate"),
        }
    }

    #[test]
    fn test_parse_l_bracket() {
        let shape = ShapeParser::parse("L-bracket flange1 50 flange2 30 width 40 thickness 3").unwrap();
        match shape {
            ShapeDescription::LBracket { flange1_length, flange2_length, width, thickness } => {
                assert!((flange1_length - 50.0).abs() < 1e-6);
                assert!((flange2_length - 30.0).abs() < 1e-6);
                assert!((width - 40.0).abs() < 1e-6);
                assert!((thickness - 3.0).abs() < 1e-6);
            }
            _ => panic!("Expected LBracket"),
        }
    }

    #[test]
    fn test_parse_no_keyword_fails() {
        let result = ShapeParser::parse("make something big");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_bare_numbers() {
        // Two bare numbers → radius + height
        let shape = ShapeParser::parse("cylinder 10 50").unwrap();
        match shape {
            ShapeDescription::Cylinder { radius, height } => {
                assert!((radius - 10.0).abs() < 1e-6);
                assert!((height - 50.0).abs() < 1e-6);
            }
            _ => panic!("Expected Cylinder"),
        }
    }

    #[test]
    fn test_shape_name() {
        assert_eq!(ShapeDescription::Box { width: 1.0, height: 1.0, depth: 1.0 }.shape_name(), "Box");
        assert_eq!(ShapeDescription::Sphere { radius: 1.0 }.shape_name(), "Sphere");
        assert_eq!(ShapeDescription::Cylinder { radius: 1.0, height: 1.0 }.shape_name(), "Cylinder");
    }

    #[test]
    fn test_dimensions_map() {
        let shape = ShapeDescription::Box { width: 100.0, height: 50.0, depth: 25.0 };
        let dims = shape.dimensions();
        assert_eq!(dims.len(), 3);
        assert!((dims["width"] - 100.0).abs() < 1e-6);
        assert!((dims["height"] - 50.0).abs() < 1e-6);
        assert!((dims["depth"] - 25.0).abs() < 1e-6);
    }
}
