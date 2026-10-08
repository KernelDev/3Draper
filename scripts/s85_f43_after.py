#!/usr/bin/env python3
"""Session-85: f43 after-mesh anatomy — the zipper's 16 new fold pairs.

For each (43,43) fold pair: 3D positions of the 6 verts, which UV region
(wall/bottom/plateau), edge lengths, and the winding consistency.
"""
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

d = Path(__file__).resolve().parent.parent / "s85_out" / "after"
objp = d / "brep2_SHAFT_SLEEVE (BREP#32629).obj"
fmapp = d / "brep2_SHAFT_SLEEVE (BREP#32629).fmap"

verts, tris = [], []
for line in objp.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))
fid_of_tri = {}
for line in fmapp.read_text().splitlines():
    p = line.split()
    if p[0] == "t":
        fid_of_tri[int(p[1])] = int(p[2])

f43 = [ti for ti, f in fid_of_tri.items() if f == 43]
print(f"f43: {len(f43)} tris, verts={len({v for ti in f43 for v in tris[ti]})}")

# parse the after (43,43) fold lines
pairs = []
for ln in open(Path(__file__).resolve().parent.parent / "s85_out" / "sleeve_pairs_after.txt"):
    if f"faces=(43,43)" not in ln:
        continue
    m = re.search(r"tris=\(\[(\d+), (\d+), (\d+)\],\[(\d+), (\d+), (\d+)\]\).*?h=\(([\d.e+-]+),([\d.e+-]+)\)", ln)
    cls = "SUBTOL" in ln
    w = "WINDING" in ln
    if m:
        t1 = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
        t2 = (int(m.group(4)), int(m.group(5)), int(m.group(6)))
        pairs.append((t1, t2, float(m.group(7)), float(m.group(8)), w, cls))

print(f"fold pairs found: {len(pairs)}")


def area2d(a, b, c):
    (ax, ay), (bx, by), (cx, cy) = a[:2], b[:2], c[:2]
    return (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)


def edge(a, b):
    return math.dist(verts[a], verts[b])


for t1, t2, h1, h2, wind, sub in pairs[:20]:
    vs = sorted(set(t1) | set(t2))
    poss = " ".join(f"v{v}=({verts[v][0]:+.3f},{verts[v][1]:+.3f},{verts[v][2]:+.3f})" for v in vs[:4])
    e_shared = [e for e in [(t1[0], t1[1]), (t1[1], t1[2]), (t1[2], t1[0])] if set(e) & set(t2)]
    lens = [edge(*t1[i:i + 2] if False else (t1[i], t1[(i + 1) % 3])) for i in range(3)]
    z = sum(verts[v][2] for v in vs) / len(vs)
    r = sum(math.hypot(verts[v][0], verts[v][1]) for v in vs) / len(vs)
    print(f"{'WIND' if wind else 'FOLD'}{' SUB' if sub else ''} h=({h1:.3f},{h2:.3f}) "
          f"z={z:.3f} r={r:.3f} lens1=[{lens[0]:.4f},{lens[1]:.4f},{lens[2]:.4f}] {poss}")

# f43 z/r ranges
zs = [verts[v][2] for ti in f43 for v in tris[ti]]
rs = [math.hypot(verts[v][0], verts[v][1]) for ti in f43 for v in tris[ti]]
print(f"\nf43 extent: z=[{min(zs):.3f},{max(zs):.3f}] r=[{min(rs):.3f},{max(rs):.3f}]")

# winding consistency census over ALL f43 tris (vs z-up normal heuristics:
# cone surface normal points outward-up; use consistent reference = fan
# around centroid)
flips = 0
tot = 0
for ti in f43:
    a, b, c = tris[ti]
    tot += 1
print(f"(winding census needs surface normals — skipped, {tot} tris)")
