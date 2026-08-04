#!/usr/bin/env python3
"""Parse navicube.obj and generate Rust constants for the ViewCube widget.

The OBJ has 26 named objects (zones):
  6 faces, 12 edges, 8 corners — each with named triangles.

Outputs Rust source with:
  - NAVICUBE_VERTS: 48 vertices [x,y,z]
  - NAVICUBE_NORMALS: 26 zone normals (one per zone)
  - NAVICUBE_ZONES: 26 zone descriptors (name, normal, label, orientation, triangle indices)
  - NAVICUBE_TRIS: all triangles (vertex indices + zone_id)
"""
import re
import sys

def parse_obj(path):
    verts = []  # [[x,y,z], ...] (1-indexed in OBJ)
    normals = []  # [[nx,ny,nz], ...] (1-indexed in OBJ)
    zones = []  # [{name, normal_idx, tris: [[v1,v2,v3], ...]}, ...]

    current_zone = None
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith('v '):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith('vn '):
                parts = line.split()
                normals.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith('o '):
                if current_zone is not None:
                    zones.append(current_zone)
                current_zone = {'name': line[2:].strip(), 'tris': [], 'normal': None}
            elif line.startswith('f '):
                # f v//n v//n v//n
                parts = line[2:].split()
                tri = []
                normal_idx = None
                for p in parts:
                    bits = p.split('//')
                    vi = int(bits[0])
                    ni = int(bits[1]) if len(bits) > 1 else None
                    tri.append(vi)
                    if ni is not None:
                        normal_idx = ni
                if current_zone is None:
                    current_zone = {'name': 'default', 'tris': [], 'normal': None}
                current_zone['tris'].append(tri)
                if normal_idx is not None:
                    current_zone['normal'] = normals[normal_idx - 1]
    if current_zone is not None:
        zones.append(current_zone)
    return verts, normals, zones

# Map zone name → (label, orientation)
ZONE_META = {
    'Right_Face':  ('RIGHT',  'Right'),
    'Left_Face':   ('LEFT',   'Left'),
    'Front_Face':  ('FRONT',  'Front'),
    'Back_Face':   ('BACK',   'Back'),
    'Top_Face':    ('TOP',    'Top'),
    'Bottom_Face': ('BOT',    'Bottom'),
    # All edges → Iso
    'Right_Front_Edge':  (None, 'Iso'),
    'Right_Back_Edge':   (None, 'Iso'),
    'Right_Top_Edge':    (None, 'Iso'),
    'Right_Bottom_Edge': (None, 'Iso'),
    'Left_Front_Edge':   (None, 'Iso'),
    'Left_Back_Edge':    (None, 'Iso'),
    'Left_Top_Edge':     (None, 'Iso'),
    'Left_Bottom_Edge':  (None, 'Iso'),
    'Front_Top_Edge':    (None, 'Iso'),
    'Front_Bottom_Edge': (None, 'Iso'),
    'Back_Top_Edge':     (None, 'Iso'),
    'Back_Bottom_Edge':  (None, 'Iso'),
    # All corners → Iso
    'TopFrontRight_Corner':    (None, 'Iso'),
    'BottomFrontRight_Corner': (None, 'Iso'),
    'TopBackRight_Corner':     (None, 'Iso'),
    'BottomBackRight_Corner':  (None, 'Iso'),
    'TopFrontLeft_Corner':     (None, 'Iso'),
    'BottomFrontLeft_Corner':  (None, 'Iso'),
    'TopBackLeft_Corner':      (None, 'Iso'),
    'BottomBackLeft_Corner':   (None, 'Iso'),
}

def main():
    verts, normals, zones = parse_obj('/tmp/tf7/navicube.obj')

    print(f"// Navicube mesh: {len(verts)} vertices, {len(zones)} zones", file=sys.stderr)
    total_tris = sum(len(z['tris']) for z in zones)
    print(f"// Total triangles: {total_tris}", file=sys.stderr)

    print("// SPDX-License-Identifier: GPL-3.0-or-later")
    print("// Auto-generated from navicube.obj — FreeCAD 0.21-style navigation cube.")
    print("// 26 named zones: 6 faces + 12 edges + 8 corners.")
    print("// Each zone has a normal, label, and set of triangles.")
    print()
    print("/// 48 vertices of the chamfered navigation cube.")
    print(f"pub const NAVICUBE_VERTS: [[f32; 3]; {len(verts)}] = [")
    for v in verts:
        print(f"    [{v[0]:.4}, {v[1]:.4}, {v[2]:.4}],")
    print("];")
    print()

    # Flatten all triangles with their zone_id
    all_tris = []  # [(v0, v1, v2, zone_id), ...]
    for zone_id, z in enumerate(zones):
        for tri in z['tris']:
            # OBJ indices are 1-based, convert to 0-based
            all_tris.append((tri[0]-1, tri[1]-1, tri[2]-1, zone_id))

    print(f"/// All triangles as (v0, v1, v2, zone_id).")
    print(f"/// zone_id indexes NAVICUBE_ZONES.")
    print(f"pub const NAVICUBE_TRIS: [(u32, u32, u32, u8); {len(all_tris)}] = [")
    for t in all_tris:
        print(f"    ({t[0]}, {t[1]}, {t[2]}, {t[3]}),")
    print("];")
    print()

    # Zone descriptors
    print("/// Zone metadata: (name, label, orientation_str, normal).")
    print("/// orientation_str is matched against ViewOrientation in code.")
    print(f"pub const NAVICUBE_ZONES: [(&str, Option<&str>, &str, [f32; 3]); {len(zones)}] = [")
    for zone_id, z in enumerate(zones):
        name = z['name']
        label, orient = ZONE_META.get(name, (None, 'Iso'))
        n = z['normal'] if z['normal'] else [0.0, 0.0, 0.0]
        label_str = f'Some("{label}")' if label else 'None'
        print(f'    ("{name}", {label_str}, "{orient}", [{n[0]:.4}, {n[1]:.4}, {n[2]:.4}]), // zone {zone_id}')
    print("];")

if __name__ == '__main__':
    main()
