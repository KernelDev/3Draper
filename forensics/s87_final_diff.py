#!/usr/bin/env python3
"""s87: diff the SLEEVE final (post-weld) mesh between baseline (s85
fallback on f41/f155) and needle-relaxed (castellation accepted) —
find WHERE the +5 (43,43) / +2 (147,147) / +2 (62,214) REAL pairs
come from: which faces' triangle sets changed post-weld."""
import sys
from collections import defaultdict

def load(base):
    obj = base + ".obj"
    fmap = base + ".fmap"
    verts = []
    for line in open(obj):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
    tri_face = {}
    for line in open(fmap):
        if line.startswith("t "):
            _, ti, fid = line.split()
            tri_face[int(ti)] = int(fid)
    tris = []
    for line in open(obj):
        if line.startswith("f "):
            _, a, b, c = line.split()
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    return verts, tris, tri_face

vb, tb, fb = load("forensics/s87_final_base/brep2_SHAFT_SLEEVE (BREP#32629)")
vr, tr, fr = load("forensics/s87_final_relax/brep2_SHAFT_SLEEVE (BREP#32629)")

print(f"baseline: {len(vb)} verts {len(tb)} tris")
print(f"relaxed : {len(vr)} verts {len(tr)} tris")

# per-face tri census
def face_census(tris, tf):
    c = defaultdict(int)
    for ti, t in enumerate(tris):
        c[tf.get(ti, -1)] += 1
    return c

cb = face_census(tb, fb)
cr = face_census(tr, fr)
allf = sorted(set(cb) | set(cr))
print("\nfaces with changed final tri counts:")
for f in allf:
    if cb.get(f, 0) != cr.get(f, 0):
        print(f"  face {f}: {cb.get(f,0)} -> {cr.get(f,0)} (delta {cr.get(f,0)-cb.get(f,0):+d})")

# the fold pairs name tri indices — map (43,43) pair tris to faces
# from the probe output the new pairs use tri ids like 515..527
print("\nbaseline tri->face for the relaxed-pair tri ids [515,518,517,151,526,524,527,149,148,150,595,161,162,167]:")
for ti in [151, 148, 149, 150, 515, 517, 518, 519, 524, 526, 527, 528, 595, 161, 162, 167]:
    if ti < len(tb):
        print(f"  base tri {ti}: face {fb.get(ti,-1)} verts {tb[ti]}")
    if ti < len(tr):
        print(f"  relax tri {ti}: face {fr.get(ti,-1)} verts {tr[ti]}")
