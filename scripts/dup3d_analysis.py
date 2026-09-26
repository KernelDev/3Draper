#!/usr/bin/env python3
"""session-61: duplicate-3D analysis of GEAR f18 pre-merge + final mesh."""
import sys
from collections import Counter

def load_obj(path):
    verts, tris = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((x, y, z))
            elif line.startswith("f "):
                a, b, c = line.split()[1:]
                tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    return verts, tris

def dup_analysis(name, verts, tris):
    cnt = Counter(verts)
    dups = {v: c for v, c in cnt.items() if c > 1}
    print(f"== {name}: {len(verts)} verts, {len(tris)} tris")
    print(f"   unique positions: {len(cnt)}, duplicated positions: {len(dups)}")
    if dups:
        top = sorted(dups.values(), reverse=True)[:5]
        print(f"   dup multiplicity top5: {top}")
        total_extra = sum(c - 1 for c in dups.values())
        print(f"   total redundant vertex slots: {total_extra}")
    # duplicated TRIANGLES (same 3 positions, any order)
    tkey = Counter()
    for t in tris:
        vs = tuple(sorted(verts[i] for i in t))
        tkey[vs] += 1
    tdupe = {k: c for k, c in tkey.items() if c > 1}
    print(f"   duplicated triangle positions: {len(tdupe)}")
    if tdupe:
        mult = Counter(tdupe.values())
        print(f"   triangle multiplicity histogram: {dict(mult)}")
    # degenerate triangles (repeated position)
    deg = sum(1 for t in tris if len(set(verts[i] for i in t)) < 3)
    print(f"   degenerate (pos-collapsed) tris: {deg}")
    return dups

# pre-merge face dump
verts, tris = load_obj("/tmp/gear_dirty/face_18_fo941.obj")
dup_analysis("f18 PRE-MERGE", verts, tris)
print()
verts, tris = load_obj("/tmp/gear_dirty/face_20_fo997.obj")
dup_analysis("f20 PRE-MERGE", verts, tris)
