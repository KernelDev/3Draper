#!/usr/bin/env python3
"""s126: diff after-gapfill vs after-winding for b10212 (p21):
list ALL triangles whose WINDING ORDER changed (the FIW cascade set),
grouped by face (fmap), with positions — to reconstruct the cascade path
and locate its ORIGIN (the triangle whose normals were already
antiparallel at after-gapfill, the FIW trigger).
"""
import os

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDIR = f"{BASE}/forensics/s126/stages2"

def load(path):
    vs, fs = [], []
    fmap = {}
    for l in open(path):
        if l.startswith("v "):
            _, x, y, z = l.split()
            vs.append((float(x), float(y), float(z)))
        elif l.startswith("f "):
            _, a, b, c = l.split()
            fs.append((int(a)-1, int(b)-1, int(c)-1))
    fmp = path.replace(".obj", ".fmap")
    if os.path.exists(fmp):
        for l in open(fmp):
            _, t, fid = l.split()
            fmap[int(t)] = int(fid)
    return vs, fs, fmap

A = load(f"{SDIR}/p21_b10212-d-after-gapfill.obj")
B = load(f"{SDIR}/p21_b10212-d-after-winding.obj")
va, fa, mapa = A
vb, fb, mapb = B
print(f"after-gapfill: v={len(va)} t={len(fa)}")
print(f"after-winding: v={len(vb)} t={len(fb)}")
if len(fa) != len(fb):
    print("TRI COUNT CHANGED between gapfill and winding?!")
# position-keyed comparison: the vertex arrays should be identical
# (FIW only reorders). Match by position.
same = len(va) == len(vb) and all(
    abs(p[0]-q[0]) < 1e-9 and abs(p[1]-q[1]) < 1e-9 and abs(p[2]-q[2]) < 1e-9
    for p, q in zip(va, vb))
print(f"vertex arrays identical: {same}")

changed = []
for i, (ta, tb) in enumerate(zip(fa, fb)):
    if ta != tb:
        changed.append(i)
print(f"\nFIW swapped {len(changed)} triangles: {changed}")
# face grouping + winding detail
from collections import defaultdict
byface = defaultdict(list)
for i in changed:
    byface[mapa.get(i, -1)].append(i)
for fid in sorted(byface):
    tis = byface[fid]
    print(f"\n face {fid}: {len(tis)} swaps")
    for i in tis[:40]:
        ta, tb = fa[i], fb[i]
        # winding change type: swap of two, or full cycle?
        if ta == (tb[1], tb[2], tb[0]) or ta == (tb[2], tb[0], tb[1]):
            kind = "cycle (same orientation!?)"
        else:
            kind = "REVERSED"
        print(f"   t{i}: {ta} -> {tb}  {kind}")
# check: are the changed triangles a connected run? report the edge
# adjacencies among them
