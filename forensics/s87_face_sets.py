#!/usr/bin/env python3
"""s87: compare face-43 and face-155 triangle SETS (not counts) between
baseline and relaxed finals — did the f41 change reshuffle f43's
connectivity through the weld?"""
from collections import defaultdict

def load(base):
    verts = []
    tris = []
    for line in open(base + ".obj"):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            _, a, b, c = line.split()
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    tf = {}
    for line in open(base + ".fmap"):
        if line.startswith("t "):
            _, ti, fid = line.split()
            tf[int(ti)] = int(fid)
    return verts, tris, tf

vb, tb, fb = load("forensics/s87_final_base/brep2_SHAFT_SLEEVE (BREP#32629)")
vr, tr, fr = load("forensics/s87_final_relax/brep2_SHAFT_SLEEVE (BREP#32629)")

for face in (41, 43, 155, 39, 147):
    sb = set()
    sr = set()
    for ti, t in enumerate(tb):
        if fb.get(ti) == face:
            sb.add(tuple(sorted(t)))
    for ti, t in enumerate(tr):
        if fr.get(ti) == face:
            sr.add(tuple(sorted(t)))
    common = sb & sr
    print(f"face {face}: base={len(sb)} relax={len(sr)} common={len(common)} "
          f"base_only={len(sb-common)} relax_only={len(sr-common)}")

# geometry shift: are vertex IDs consistent between runs? Compare a few
# specific vertices used by the (43,43) pairs: 515,517,518,526,524,527
print("\nvertex positions (id: base -> relax):")
for vid in (341, 332, 515, 517, 518, 524, 526, 527):
    print(f"  v{vid}: {vb[vid]} -> {vr[vid]} "
          f"{'SAME' if vb[vid]==vr[vid] else 'DIFF'}")

# vertex count diff — which id disappeared?
print(f"\nverts: base={len(vb)} relax={len(vr)}")
n = min(len(vb), len(vr))
diffs = [i for i in range(n) if vb[i] != vr[i]]
print(f"first 10 differing vertex ids: {diffs[:10]} (total {len(diffs)})")
