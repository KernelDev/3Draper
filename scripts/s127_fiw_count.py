#!/usr/bin/env python3
"""s127 п.4: FIW flip count on #10212 AFTER the emission fix.
Compare after-gapfill vs after-winding triangle windings: count triangles
whose (sorted) vertex triple matches but whose order was FLIPPED.
s126 baseline: 1772/1772 flips (47% of the mesh).
"""
import sys

def load(path):
    verts, tris = [], []
    for l in open(path):
        p = l.split()
        if not p:
            continue
        if p[0] == "v":
            verts.append(tuple(float(x) for x in p[1:4]))
        elif p[0] == "f":
            idx = [int(x.split("/")[0]) - 1 for x in p[1:4]]
            tris.append(tuple(idx))
    return verts, tris

def analyze(tag_pre, tag_post):
    a_verts, a_tris = load(f"forensics/s127/stages_10212/p21_b10212-d-after-gapfill.obj")
    b_verts, b_tris = load(f"forensics/s127/stages_10212/p21_b10212-d-after-winding.obj")
    print(f"{tag_pre}: verts={len(a_verts)} tris={len(a_tris)}")
    print(f"{tag_post}: verts={len(b_verts)} tris={len(b_tris)}")
    # map by sorted vertex triple (positions may renumber; use position keys)
    def key_of(verts, t):
        return tuple(sorted(tuple(round(c, 6) for c in verts[i]) for i in t))
    a_map = {}
    for t in a_tris:
        a_map.setdefault(key_of(a_verts, t), []).append(t)
    b_map = {}
    for t in b_tris:
        b_map.setdefault(key_of(b_verts, t), []).append(t)
    common = set(a_map) & set(b_map)
    print(f"matched-by-position triangles: {len(common)} of A={len(a_map)} B={len(b_map)}")
    flipped = 0
    same = 0
    # orientation compare via centroid-normal (positions equal, order may differ)
    def orient(verts, t):
        a, b, c = (verts[i] for i in t)
        u = tuple(b[k]-a[k] for k in range(3))
        v = tuple(c[k]-a[k] for k in range(3))
        return (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
    for k in common:
        ta, tb = a_map[k][0], b_map[k][0]
        oa = orient(a_verts, ta)
        ob = orient(b_verts, tb)
        dot = sum(oa[i]*ob[i] for i in range(3))
        mag = (sum(x*x for x in oa) * sum(x*x for x in ob)) ** 0.5
        if mag > 1e-30:
            cos = dot / mag
            if cos < 0:
                flipped += 1
            else:
                same += 1
    print(f"FIW flips (gapfill->winding, position-matched): {flipped}, same-orientation: {same}")
    pct = 100.0 * flipped / max(len(b_tris), 1)
    print(f"flip share of final mesh: {pct:.1f}%  (s126 baseline: 1772 flips = 47%)")

analyze("after-gapfill", "after-winding")
