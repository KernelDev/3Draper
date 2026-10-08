#!/usr/bin/env python3
"""Session-85: UV anatomy of Cone f155 (brep32629) — why does earcut
fan from ring points across the domain?

Dump: 1872 boundary pts, 0 holes, 7 interior, 1870 tris.
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

p = Path(sys.argv[1] if len(sys.argv) > 1 else "s85_out/tri/tri_0080_Cone.txt")
lines = p.read_text().splitlines()
hdr = lines[0]
mode = None
bnd, holes, interior, tris = [], defaultdict(list), [], []
for ln in lines[1:]:
    parts = ln.split()
    if not parts:
        continue
    if parts[0] in ("boundary", "interior", "tris") or parts[0].startswith("hole"):
        mode = parts[0]
        if parts[0].startswith("hole"):
            mode = "hole"
        continue
    if mode == "boundary":
        bnd.append((float(parts[1]), float(parts[2])))
    elif mode == "hole":
        holes[int(parts[1])].append((float(parts[1+1]), float(parts[2+1])))
    elif mode == "interior":
        interior.append((float(parts[1]), float(parts[2])))
    elif mode == "tris":
        tris.append((int(parts[1]), int(parts[2]), int(parts[3])))

n_b = len(bnd)
n_i = len(interior)
print(f"{hdr}")
print(f"bnd={n_b} holes={len(holes)} interior={n_i} tris={len(tris)}")

us = [q[0] for q in bnd]
vs = [q[1] for q in bnd]
print(f"boundary u: [{min(us):.4f},{max(us):.4f}]  v: [{min(vs):.4f},{max(vs):.4f}]")
dup = len(us) - len(set(bnd))
print(f"exact duplicate boundary points: {dup}")

# fan detection in index space
cnt = defaultdict(int)
for t in tris:
    for v in t:
        cnt[v] += 1
top = sorted(cnt.items(), key=lambda kv: -kv[1])[:12]
print("top-degree vertex indices:", top)

# are boundary pts duplicated as interior? bnd base = 0..n_b-1, interior = n_b..
print("interior indices:", [n_b + k for k in range(n_i)])

# u-wrap structure: consecutive runs where u jumps back (seam crossings)
seam_jumps = []
for k in range(n_b):
    a, b = bnd[k], bnd[(k + 1) % n_b]
    du = b[0] - a[0]
    if abs(du) > (max(us) - min(us)) * 0.5:
        seam_jumps.append(k)
print(f"seam-scale u jumps at boundary idx: {seam_jumps[:10]} (n={len(seam_jumps)})")

# v-structure: histogram of v values
vh = defaultdict(int)
for _, vv in bnd:
    vh[round(vv, 3)] += 1
print("v histogram (top 12):", sorted(vh.items(), key=lambda kv: -kv[1])[:12])

# u-structure: how many points per u value (multi-cover detection)
uh = defaultdict(int)
for uu, _ in bnd:
    uh[round(uu, 4)] += 1
multi = sorted(uh.items(), key=lambda kv: -kv[1])[:8]
print("u-value multiplicity (top 8):", multi)

# self-intersection quick check on the first 300 pts (shallow)
def seg_int(p1, p2, p3, p4):
    d1 = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0])
    d2 = (p2[0]-p1[0])*(p4[1]-p1[1]) - (p2[1]-p1[1])*(p4[0]-p1[0])
    d3 = (p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])
    d4 = (p4[0]-p3[0])*(p2[1]-p3[1]) - (p4[1]-p3[1])*(p2[0]-p3[0])
    return d1*d2 < 0 and d3*d4 < 0

# full O(n^2) on 1872 = 3.5M pairs — fine in a few seconds
intersections = 0
examples = []
nseg = n_b
for i in range(nseg):
    p1, p2 = bnd[i], bnd[(i + 1) % nseg]
    for j in range(i + 2, nseg):
        if i == 0 and j == nseg - 1:
            continue
        p3, p4 = bnd[j], bnd[(j + 1) % nseg]
        if seg_int(p1, p2, p3, p4):
            intersections += 1
            if len(examples) < 6:
                examples.append((i, j, p1, p3))
print(f"boundary self-intersections: {intersections}")
for e in examples:
    print(f"  seg {e[0]} ({e[2][0]:.3f},{e[2][1]:.3f}) x seg {e[1]} ({e[3][0]:.3f},{e[3][1]:.3f})")

# fan apex analysis: the top-degree boundary indices — what are their neighbors?
for apex, deg in top[:3]:
    if apex >= n_b:
        continue
    print(f"\napex idx {apex} (bnd pt {bnd[apex]}, deg={deg}):")
    # tris containing apex
    ts = [t for t in tris if apex in t]
    others = set()
    for t in ts:
        others.update(v for v in t if v != apex)
    others_b = sorted(v for v in others if v < n_b)
    print(f"  in {len(ts)} tris; other bnd idx span {min(others_b)}..{max(others_b)} ({len(others_b)} distinct)")
