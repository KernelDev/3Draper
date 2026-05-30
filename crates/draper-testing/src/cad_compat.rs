// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 KernelDev
//! T.7 — CAD System Compatibility
//!
//! Detect originating CAD system from STEP header and test compatibility.

/// Known CAD systems that produce STEP files.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CadSystem {
    SolidWorks,
    CATIA,
    NX,
    Inventor,
    FreeCAD,
    Unknown,
}

impl std::fmt::Display for CadSystem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CadSystem::SolidWorks => write!(f, "SolidWorks"),
            CadSystem::CATIA => write!(f, "CATIA"),
            CadSystem::NX => write!(f, "NX"),
            CadSystem::Inventor => write!(f, "Inventor"),
            CadSystem::FreeCAD => write!(f, "FreeCAD"),
            CadSystem::Unknown => write!(f, "Unknown"),
        }
    }
}

/// Result of testing a STEP file for CAD compatibility.
#[derive(Debug)]
pub struct CadCompatResult {
    /// Filename of the STEP file.
    pub filename: String,
    /// Detected originating CAD system.
    pub detected_cad: CadSystem,
    /// Whether parsing succeeded.
    pub parse_ok: bool,
    /// Whether triangulation succeeded.
    pub triangulate_ok: bool,
    /// Any warnings about compatibility.
    pub warnings: Vec<String>,
}

/// Detect the originating CAD system from STEP file header content.
///
/// Examines the FILE_NAME, FILE_DESCRIPTION, and other header entities
/// for known patterns:
/// - SolidWorks: typically contains "SolidWorks" in FILE_NAME or FILE_DESCRIPTION
/// - CATIA: typically contains "CATIA" in header
/// - NX (Siemens): typically contains "NX" or "Unigraphics" in header
/// - Inventor (Autodesk): typically contains "Inventor" or "Autodesk" in header
/// - FreeCAD: typically contains "FreeCAD" in header
pub fn detect_cad_system(step_content: &str) -> CadSystem {
    let content_upper = step_content.to_uppercase();

    // Check for known CAD system patterns in the header
    // The HEADER section is between "HEADER;" and "ENDSEC;"
    let header_end = content_upper.find("ENDSEC;").unwrap_or(content_upper.len());
    let header = &content_upper[..header_end];

    if header.contains("SOLIDWORKS") {
        return CadSystem::SolidWorks;
    }
    if header.contains("CATIA") {
        return CadSystem::CATIA;
    }
    if header.contains("NX") || header.contains("UNIGRAPHICS") {
        return CadSystem::NX;
    }
    if header.contains("INVENTOR") || header.contains("AUTODESK") {
        return CadSystem::Inventor;
    }
    if header.contains("FREECAD") {
        return CadSystem::FreeCAD;
    }

    CadSystem::Unknown
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_solidworks() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('SolidWorks Model'),'2;1');
FILE_NAME('test.stp','','',('SolidWorks'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
"#;
        assert_eq!(detect_cad_system(content), CadSystem::SolidWorks);
    }

    #[test]
    fn test_detect_catia() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('CATIA Model'),'2;1');
FILE_NAME('test.stp','','',('CATIA'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
"#;
        assert_eq!(detect_cad_system(content), CadSystem::CATIA);
    }

    #[test]
    fn test_detect_nx() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('NX Model'),'2;1');
FILE_NAME('test.stp','','',('NX'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
"#;
        assert_eq!(detect_cad_system(content), CadSystem::NX);
    }

    #[test]
    fn test_detect_freecad() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('FreeCAD Model'),'2;1');
FILE_NAME('test.stp','','',('FreeCAD'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
"#;
        assert_eq!(detect_cad_system(content), CadSystem::FreeCAD);
    }

    #[test]
    fn test_detect_unknown() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Generic Model'),'2;1');
FILE_NAME('test.stp','','',('Unknown'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
"#;
        assert_eq!(detect_cad_system(content), CadSystem::Unknown);
    }

    #[test]
    fn test_detect_inventor() {
        let content = r#"ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('Autodesk Inventor Model'),'2;1');
FILE_NAME('test.stp','','',('Autodesk'),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
ENDSEC;
END-ISO-10303-21;
"#;
        assert_eq!(detect_cad_system(content), CadSystem::Inventor);
    }
}
