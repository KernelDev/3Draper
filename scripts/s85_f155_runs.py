#!/usr/bin/env python3
"""Session-85: run-structure of the f155 UV boundary (1872 pts).

Classify consecutive boundary points into runs: v=const (horizontal),
u=const (vertical), wavy. Print the run table to understand the comb.
"""
import sys
from pathlib import Path

p = Path(sys.argv[1] if len(sys.argv) > 1 else "s85_out/tri/tri_0080_Cone.txt")
lines = p.read_text().splitlines()
mode = None
bnd, tris = [], []
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
    elif mode == "tris":
        tris.append((int(parts[1]), int(parts[2]), int(parts[3])))

n = len(bnd)
# run classification with tolerances
EPS = 1e-4
runs = []
i = 0
# start at a corner (u=0)
start = min(range(n), key=lambda k: (bnd[k][0], bnd[k][1]))
bnd = bnd[start:] + bnd[:start]
i = 0
cur_kind = None
cur_start = 0
for k in range(1, n + 1):
    a = bnd[k - 1]
    b = bnd[k % n]
    du, dv = b[0] - a[0], b[1] - a[1]
    if abs(dv) < EPS and abs(du) > EPS:
        kind = "H"  # horizontal (v=const)
    elif abs(du) < EPS and abs(dv) > EPS:
        kind = "V"  # vertical (u=const)
    else:
        kind = "D"  # diagonal/wavy
    if cur_kind is None:
        cur_kind, cur_start = kind, k - 1
    elif kind != cur_kind:
        runs.append((cur_start, k - 1, cur_kind, bnd[cur_start], bnd[k - 1]))
        cur_kind, cur_start = kind, k - 1
runs.append((cur_start, n, cur_kind, bnd[cur_start], bnd[0]))

print(f"n={n}, runs={len(runs)}")
for s, e, kind, pa, pb in runs[:80]:
    print(f"  [{s:4d}..{e:4d}] {kind} ({pa[0]:.4f},{pa[1]:.4f})->({pb[0]:.4f},{pb[1]:.4f}) len={e-s+1}")

# fan apex 1782 in ORIGINAL indexing = (1782 - start) % n
apexes = [1782, 1815]
print("\napex positions (orig idx): 1782 (0,-0.0106), 1815 (pi,-0.0106)")
# count tris by their minimal boundary index distance from apex
import collections
deg = collections.Counter()
for t in tris:
    for v in t:
        deg[v] += 1
print("top degree:", deg.most_common(6))
