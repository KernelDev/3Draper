#!/usr/bin/env python3
"""Session-85: legacy-result stats for f155/f41 — rim coverage, extra
edges, interior usage, fan structure. Decides the acceptance-gate shape
for a castellation-zipper arm."""
import sys
from collections import defaultdict
from pathlib import Path

p = Path(sys.argv[1])
lines = p.read_text().splitlines()
mode = None
bnd, interior, tris = [], [], []
for ln in lines[1:]:
    parts = ln.split()
    if not parts:
        continue
    if parts[0] in ("boundary", "interior", "tris"):
        mode = parts[0]
        continue
    if parts[0].startswith("hole"):
        mode = "hole"
        continue
    if mode == "boundary":
        bnd.append((float(parts[1]), float(parts[2])))
    elif mode == "interior":
        interior.append((float(parts[1]), float(parts[2])))
    elif mode == "tris":
        tris.append((int(parts[1]), int(parts[2]), int(parts[3])))

n_b = len(bnd)
n_i = len(interior)
edge_cnt = defaultdict(int)
for t in tris:
    for k in range(3):
        a, b = t[k], t[(k + 1) % 3]
        if a == b:
            continue
        edge_cnt[(min(a, b), max(a, b))] += 1

rim_present = 0
for i in range(n_b):
    j = (i + 1) % n_b
    if (min(i, j), max(i, j)) in edge_cnt:
        rim_present += 1
extra = sum(1 for e, c in edge_cnt.items() if c == 1 and not (
    (e[0] < n_b and e[1] < n_b and abs(e[0] - e[1]) in (1, n_b - 1))
))
one_sided = sum(1 for c in edge_cnt.values() if c == 1)
used_int = sorted({v for t in tris for v in t if v >= n_b})
used_bnd = len({v for t in tris for v in t if v < n_b})
print(f"{p.name}: n_b={n_b} n_i={n_i} tris={len(tris)}")
print(f"  rim edges present: {rim_present}/{n_b}")
print(f"  one-sided edges total: {one_sided} (extra non-rim: {extra})")
print(f"  boundary verts used: {used_bnd}/{n_b}")
print(f"  interior used: {len(used_int)}/{n_i} idx={used_int}")
print(f"  interior pts: {interior}")

# fan metrics
deg = defaultdict(int)
for t in tris:
    for v in t:
        deg[v] += 1
fan_apexes = [(v, d) for v, d in deg.items() if d >= 40]
print(f"  apexes deg>=40: {fan_apexes}")
for v, d in sorted(fan_apexes, key=lambda x: -x[1])[:3]:
    ts = [t for t in tris if v in t]
    others = sorted({w for t in ts for w in t if w != v})
    span = others[-1] - others[0] if others else 0
    print(f"    apex {v}: deg={d} span={span}/{n_b} ({span/n_b:.0%})")
