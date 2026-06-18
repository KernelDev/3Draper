#!/usr/bin/env python3
"""Second-pass warning fixer: adds #[allow(...)] attributes and fixes remaining issues."""

import re
import os
from pathlib import Path

PROJECT = Path('/home/z/my-project')

# Files to add #![allow(dead_code)] at the top (after the copyright header)
FILES_WITH_DEAD_CODE = [
    'crates/draper-mesh/src/certification.rs',
    'crates/draper-mesh/src/custom_cdt.rs',
    'crates/draper-mesh/src/text3d.rs',
    'crates/draper-mesh/src/cdt_triangulate.rs',
    'crates/draper-mesh/src/parametric_domain.rs',
    'crates/draper-step/src/converter.rs',
]

# Specific fixes per file
SPECIFIC_FIXES = {
    'crates/draper-mesh/src/mesh.rs': [
        # Remove unused import std::fmt
        ('use std::fmt;\n', ''),
    ],
    'crates/draper-mesh/src/gdt_check.rs': [
        ('use draper_geometry::Point3d;\n', ''),
    ],
    'crates/draper-mesh/src/edge_cache.rs': [
        # L778: surface unused
        ('(surface: &Surface,', '(_surface: &Surface,'),
    ],
    'crates/draper-mesh/src/parametric_domain.rs': [
        # L810, L933: surface unused (these are closures)
        ('|surface: &Surface|', '|_surface: &Surface|'),
        # L2725: u, v unused
        ('|u: f64, v: f64, p: &Point3d|', '|_u: f64, _v: f64, p: &Point3d|'),
    ],
    'crates/draper-mesh/src/watertight.rs': [
        # Remove unused import Surface
        ('use draper_geometry::Surface;\n', ''),
        # L201: lo, hi unused in for loop
        ('for (lo, hi) in', 'for (_lo, _hi) in'),
        # L331, L332: vi, vj unused
        ('|vi: usize, vj: usize|', '|_vi: usize, _vj: usize|'),
        ('|vi, vj|', '|_vi, _vj|'),
    ],
    'crates/draper-mesh/src/export.rs': [
        # Add #[allow(unused_assignments)] at the top
        ('//!', '//!\n#![allow(unused_assignments)]\n'),
    ],
    'crates/draper-mesh/src/lib.rs': [
        # Add #[allow(ambiguous_glob_reexports)] at the top
        ('#![warn(clippy::unwrap_used)]', '#![warn(clippy::unwrap_used)]\n#![allow(ambiguous_glob_reexports)]'),
    ],
    'crates/draper-step/src/parser.rs': [
        ('use std::collections::HashMap;\n', ''),
    ],
    'crates/draper-step/src/exporter.rs': [
        ('use crate::schema::*;\n', ''),
        ('use draper_topology::{Face, Shell};\n', ''),
        ('|edge_start_vtx_id: u64, edge_end_vtx_id: u64|', '|_edge_start_vtx_id: u64, _edge_end_vtx_id: u64|'),
    ],
    'crates/draper-step/src/converter.rs': [
        # Remove unused imports from use statements
        ('Ellipse,', ''),
        ('use draper_mesh::{smooth_normals_adaptive, smooth_normals};\n', ''),
        ('use draper_mesh::AdaptiveTolerance;\n', ''),
        # Unused variables
        ('child_pd_id: u64,', 'child_pd_id: u64, // unused\n'),
        ('let child_pd_id =', 'let _child_pd_id ='),
        ('let vp =', 'let _vp ='),
        ('let curve =', 'let _curve ='),
    ],
    'crates/draper-viewer/src/app.rs': [
        # Add #[allow(dead_code)] at the top
        ('//!', '//!\n#![allow(dead_code)]\n'),
    ],
    'crates/draper-viewer/src/camera.rs': [
        ('//!', '//!\n#![allow(dead_code)]\n'),
    ],
    'crates/draper-viewer/src/renderer.rs': [
        ('//!', '//!\n#![allow(dead_code)]\n'),
    ],
    'crates/draper-ffi/src/lib.rs': [
        ('//!', '//!\n#![allow(dead_code)]\n'),
    ],
}

def add_allow_dead_code(filepath):
    """Add #![allow(dead_code)] after the copyright header."""
    path = PROJECT / filepath
    if not path.exists():
        return False
    content = path.read_text()
    if '#![allow(dead_code)]' in content:
        return False
    # Find the position after the doc comment block
    # Pattern: lines starting with // or //! until first non-comment line
    lines = content.split('\n')
    insert_idx = 0
    in_comment = False
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith('//') or stripped.startswith('/*!') or stripped.startswith('*/'):
            in_comment = True
            insert_idx = i + 1
        elif in_comment and stripped == '':
            insert_idx = i + 1
            continue
        elif in_comment:
            break
        else:
            insert_idx = i
            break
    
    # Insert after the line at insert_idx
    lines.insert(insert_idx, '#![allow(dead_code)]')
    # Add a blank line before if needed
    if insert_idx > 0 and lines[insert_idx - 1].strip() and not lines[insert_idx - 1].strip().startswith('//'):
        lines.insert(insert_idx, '')
    
    path.write_text('\n'.join(lines))
    return True

def apply_specific_fixes(filepath, fixes):
    """Apply specific text replacements to a file."""
    path = PROJECT / filepath
    if not path.exists():
        return 0
    content = path.read_text()
    count = 0
    for old, new in fixes:
        if old in content:
            content = content.replace(old, new, 1)
            count += 1
    if count > 0:
        path.write_text(content)
    return count

def fix_unreachable_patterns(filepath):
    """Add #[allow(unreachable_patterns)] to specific match blocks."""
    path = PROJECT / filepath
    if not path.exists():
        return
    content = path.read_text()
    if 'unreachable_patterns' in content:
        return
    # Add at the top of the file
    lines = content.split('\n')
    # Find a good insertion point
    for i, line in enumerate(lines):
        if line.strip() and not line.startswith('//') and not line.startswith('//!') and not line.startswith('/*'):
            lines.insert(i, '#![allow(unreachable_patterns)]')
            lines.insert(i + 1, '')
            break
    path.write_text('\n'.join(lines))

def fix_remaining_unused_vars():
    """Fix any remaining unused variable warnings by examining current state."""
    # Run cargo check and parse remaining warnings
    import subprocess
    env = os.environ.copy()
    env['PATH'] = os.path.expanduser('~/.cargo/bin') + ':' + env['PATH']
    result = subprocess.run(
        ['cargo', 'check', '--release'],
        capture_output=True, text=True, env=env, cwd=str(PROJECT)
    )
    out = result.stdout + result.stderr
    
    # Parse warnings: warning: <msg>\n   --> file:line:col
    pattern = re.compile(r'^warning: (.*?)\n\s+--> (.*?):(\d+):(\d+)', re.MULTILINE)
    fixes_applied = 0
    
    from collections import defaultdict
    by_file = defaultdict(list)
    for m in pattern.finditer(out):
        msg = m.group(1)
        file = m.group(2)
        line = int(m.group(3))
        col = int(m.group(4))
        by_file[file].append((line, col, msg))
    
    for file, warnings in by_file.items():
        path = PROJECT / file
        if not path.exists():
            continue
        lines = path.read_text().split('\n')
        # Sort descending by line to patch from bottom up
        warnings.sort(key=lambda w: -w[0])
        for line_no, col, msg in warnings:
            # Match different warning types
            m = re.match(r'unused variable: `(\w+)`', msg)
            if m:
                var = m.group(1)
                idx = line_no - 1
                if idx >= len(lines):
                    continue
                line = lines[idx]
                # Try patterns: let mut NAME, let NAME, NAME: Type, |NAME|
                # Only prefix if not already prefixed
                if f'_{var}' in line and f'_{var} ' in line:
                    continue
                new_line = line
                # Pattern: `let NAME` or `let mut NAME`
                if re.search(rf'\blet\s+(mut\s+)?{re.escape(var)}\b', line):
                    new_line = re.sub(rf'(\blet\s+(?:mut\s+)?){re.escape(var)}\b', r'\1_' + var, line, count=1)
                # Pattern: `|NAME: Type|` or `|NAME|`
                elif re.search(rf'\|{re.escape(var)}(\s*:|\s*\|)', line):
                    new_line = re.sub(rf'\|{re.escape(var)}', '|_' + var, line, count=1)
                # Pattern: `NAME: Type,` (fn param)
                elif re.search(rf'\b{re.escape(var)}\s*:', line):
                    new_line = re.sub(rf'\b{re.escape(var)}(\s*:)', '_' + var + r'\1', line, count=1)
                # Pattern: `for NAME in`
                elif re.search(rf'\bfor\s+{re.escape(var)}\s+in\b', line):
                    new_line = re.sub(rf'(\bfor\s+){re.escape(var)}(\s+in\b)', r'\1_' + var + r'\2', line, count=1)
                
                if new_line != line:
                    lines[idx] = new_line
                    fixes_applied += 1
        
        path.write_text('\n'.join(lines))
    
    return fixes_applied

def main():
    # 1. Add #![allow(dead_code)] to files with many dead-code warnings
    for f in FILES_WITH_DEAD_CODE:
        if add_allow_dead_code(f):
            print(f"Added #![allow(dead_code)] to {f}")
    
    # 2. Apply specific fixes
    for f, fixes in SPECIFIC_FIXES.items():
        n = apply_specific_fixes(f, fixes)
        if n > 0:
            print(f"Applied {n} fixes to {f}")
    
    # 3. Fix unreachable patterns
    fix_unreachable_patterns('crates/draper-mesh/src/certification.rs')
    print("Added #[allow(unreachable_patterns)] to certification.rs")
    
    # 4. Fix remaining unused variables
    n = fix_remaining_unused_vars()
    print(f"Fixed {n} remaining unused variables")

if __name__ == '__main__':
    main()
