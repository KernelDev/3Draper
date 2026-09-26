#!/usr/bin/env python3
"""Session-57: attribute FO-pair edge endpoints to tolerance-weld targets.

Reads the TOLWELD dump (from DRAPPER_DUMP_WELDS=1) and the final OBJ+fmap,
then for every topo-consistent same-side >170° pair (FOLD-OVER) checks
whether its shared-edge endpoints are weld TARGETS (a tolerance weld
remapped some incoming vertex onto that global index, deforming the
incoming triangle by the weld distance d).

Usage: weld_fo_link.py <welds.txt> <final_dir>
"""
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

welds_file = Path(sys.argv[1])
d = Path(sys.argv[2])
obj = next(d.glob("*.obj"))
fmap = obj.with_suffix(".fmap")

# Parse welds: existing_idx -> list of distances
weld_targets = defaultdict(list)  # idx -> [d, ...]
pat = re.compile(r"->(\d+) d=([0-9.]+)")
for line in welds_file.read_text().splitlines():
    m = pat.search(line)
    if m:
        weld_targets[int(m.group(1))].append(float(m.group(2)))

deforming = {i for i, ds in weld_targets.items() if max(ds) > 1e-5}

verts, tris = [], []
for line in obj.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))
fid_of_tri = {}
for line in fmap.read_text().splitlines():
    if line.startswith("t "):
        _, ti, fid = line.split()
        fid_of_tri[int(ti)] = int(fid)


def sub(p, q):
    return (p[0] - q[0], p[1] - q[1], p[2] - q[2])


def cross(p, q):
    return (p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0])


def dot(p, q):
    return p[0] * q[0] + p[1] * q[1] + p[2] * q[2]


def tri_normal(t):
    n = cross(sub(verts[t[1]], verts[t[0]]), sub(verts[t[2]], verts[t[0]]))
    l = math.sqrt(dot(n, n))
    return (n[0] / l, n[1] / l, n[2] / l) if l > 1e-30 else None


normals = [tri_normal(t) for t in tris]
edge_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(tris):
    for e in ((a, b), (b, c), (c, a)):
        edge_tris[(min(e), max(e))].append((ti, e[0], e[1]))

stats = defaultdict(lambda: [0, 0, 0])  # cls -> [total, with_def, with_any]
weld_d_at_edge = []
for (e, tl) in edge_tris.items():
    if len(tl) != 2:
        continue
    (t0, a0, b0), (t1, a1, b1) = tl
    n0, n1 = normals[t0], normals[t1]
    if not n0 or not n1:
        continue
    ang = math.degrees(math.acos(max(-1.0, min(1.0, dot(n0, n1)))))
    if ang <= 170.0:
        continue
    f0, f1 = fid_of_tri.get(t0, -1), fid_of_tri.get(t1, -1)
    topo_consistent = (a0 == b1) and (b0 == a1)
    a, b = verts[e[0]], verts[e[1]]
    ed = sub(b, a)

    def apex_side(ti_, e0, e1):
        for v in tris[ti_]:
            if v != e0 and v != e1:
                return cross(ed, sub(verts[v], a))
        return (0.0, 0.0, 0.0)

    s0, s1 = apex_side(t0, e[0], e[1]), apex_side(t1, e[0], e[1])
    same_side = dot(s0, s1) > 0
    if not (topo_consistent and same_side):
        continue
    cls = "FO-sameface" if f0 == f1 else "FO-crossface"
    stats[cls][0] += 1
    hit_def = any(v in deforming for v in e)
    hit_any = any(v in weld_targets for v in e)
    if hit_def:
        stats[cls][1] += 1
        ds = [dd for v in e for dd in weld_targets.get(v, []) if dd > 1e-5]
        weld_d_at_edge.extend(ds)
    if hit_any:
        stats[cls][2] += 1

print("class          total  edge∈def-weld-target  edge∈any-weld-target")
for cls, (tot, wd, wa) in sorted(stats.items()):
    pct = 100.0 * wd / tot if tot else 0
    print(f"{cls:<14} {tot:>6}  {wd:>6} ({pct:.0f}%)          {wa:>6}")

if weld_d_at_edge:
    weld_d_at_edge.sort()
    n = len(weld_d_at_edge)
    print(f"\nweld d at FO edges (n={n}): min={weld_d_at_edge[0]:.2e} "
          f"med={weld_d_at_edge[n//2]:.2e} max={weld_d_at_edge[-1]:.2e}")

print(f"\nweld targets total: {len(weld_targets)}, deforming (d>1e-5): {len(deforming)}")
