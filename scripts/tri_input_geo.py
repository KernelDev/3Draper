#!/usr/bin/env python3
"""Session-58: analyze a DRAPPER_DUMP_TRI_INPUT dump (UV-level).

Structure of the face triangulation inputs: ring convexity, interior
lattice shape (full-rect? ragged? chain order), triangle quality
(fans, monsters, cross-strip ears), coverage census.

Usage: tri_input_geo.py <dump.txt>
"""
import sys
from collections import defaultdict
from pathlib import Path

src = Path(sys.argv[1])
mode = None
bnd, holes, inter, tris = [], defaultdict(list), [], []
cur_hole = None
for line in src.read_text().splitlines():
    f = line.split()
    if not f:
        continue
    if f[0] == "type":
        print(line)
        continue
    if f[0] == "boundary":
        mode = "b"
        continue
    if f[0] == "hole":
        cur_hole = int(f[1])
        mode = "h"
        continue
    if f[0] == "interior":
        mode = "i"
        continue
    if f[0] == "tris":
        mode = "t"
        continue
    if mode == "b":
        bnd.append((float(f[1]), float(f[2])))
    elif mode == "h":
        holes[cur_hole].append((float(f[1]), float(f[2])))
    elif mode == "i":
        inter.append((float(f[1]), float(f[2])))
    elif mode == "t":
        tris.append((int(f[1]), int(f[2]), int(f[3])))

n_b = len(bnd)
allp = bnd + [p for h in holes.values() for p in h] + inter
print(f"boundary={n_b} holes={sum(len(h) for h in holes.values())} "
      f"interior={len(inter)} tris={len(tris)}")

# ── 1. Ring: bbox, convexity (cross signs), edge lengths ──────────
us = [p[0] for p in bnd]
vs = [p[1] for p in bnd]
print(f"ring bbox: u=[{min(us):.4f},{max(us):.4f}] v=[{min(vs):.4f},{max(vs):.4f}]")

def cross2(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])

pos = neg = 0
min_area = 1e18
for i in range(n_b):
    c = cross2(bnd[i], bnd[(i+1) % n_b], bnd[(i+2) % n_b])
    min_area = min(min_area, abs(c))
    if c > 1e-12:
        pos += 1
    elif c < -1e-12:
        neg += 1
print(f"ring convexity: pos={pos} neg={neg} straight={n_b-pos-neg} "
      f"min cross={min_area:.3e}  -> {'CONVEX' if pos == 0 or neg == 0 else 'NON-CONVEX'}")

ring_edges = []
for i in range(n_b):
    a, b = bnd[i], bnd[(i+1) % n_b]
    ring_edges.append(((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5)
ring_edges_sorted = sorted(ring_edges)
print(f"ring edge len: min={ring_edges_sorted[0]:.3e} p25={ring_edges_sorted[n_b//4]:.3e} "
      f"med={ring_edges_sorted[n_b//2]:.3e} p75={ring_edges_sorted[3*n_b//4]:.3e} "
      f"max={ring_edges_sorted[-1]:.3e}")

# ── 2. Interior lattice: cluster u/v values, full-rect check ──────
def cluster(vals, tol=1e-9):
    s = sorted(vals)
    out = [[s[0]]]
    for x in s[1:]:
        if x - out[-1][-1] > tol:
            out.append([x])
        else:
            out[-1].append(x)
    return out

uc = cluster([p[0] for p in inter])
vc = cluster([p[1] for p in inter])
print(f"lattice clusters: n_u={len(uc)} n_v={len(vc)} "
      f"product={len(uc)*len(vc)} (interior={len(inter)})")
u_vals = [c[0] for c in uc]
v_vals = [c[0] for c in vc]
if len(uc) * len(vc) == len(inter):
    print(f"  FULL RECT GRID: u {len(uc)} cols v {len(vc)} rows")
    du = [u_vals[i+1]-u_vals[i] for i in range(len(u_vals)-1)]
    dv = [v_vals[i+1]-v_vals[i] for i in range(len(v_vals)-1)]
    print(f"  u spacing: min={min(du):.4f} med={sorted(du)[len(du)//2]:.4f} "
          f"max={max(du):.4f}  v spacing: min={min(dv):.4f} "
          f"med={sorted(dv)[len(dv)//2]:.4f} max={max(dv):.4f}")
    print(f"  u vals: {[round(x,3) for x in u_vals[:26]]}")
    print(f"  v vals: {[round(x,3) for x in v_vals[:26]]}")

# ── 3. Triangles: classify by index ranges ────────────────────────
def kind(i):
    if i < n_b:
        return "B"
    if i < n_b + sum(len(h) for h in holes.values()):
        return "H"
    return "I"

kinds = defaultdict(int)
for t in tris:
    kinds["".join(sorted(kind(i) for i in t))] += 1
print("tri kinds:", dict(sorted(kinds.items())))

# ring-edge presence: which consecutive ring pairs are edges of a tri?
edge_set = set()
for t in tris:
    for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        edge_set.add((min(a, b), max(a, b)))
missing = [i for i in range(n_b)
           if (min(i, (i+1) % n_b), max(i, (i+1) % n_b)) not in edge_set]
print(f"missing ring edges: {len(missing)}/{n_b} at {missing[:8]}")

# ── 4. Monsters: triangle UV bbox spans vs lattice steps ──────────
med_du = sorted([abs(u_vals[i+1]-u_vals[i]) for i in range(len(u_vals)-1)])[len(u_vals)//2] if len(u_vals) > 1 else 1.0
med_dv = sorted([abs(v_vals[i+1]-v_vals[i]) for i in range(len(v_vals)-1)])[len(v_vals)//2] if len(v_vals) > 1 else 1.0
spans = []
for t in tris:
    tu = [allp[i][0] for i in t]
    tv = [allp[i][1] for i in t]
    spans.append((max(tu)-min(tu), max(tv)-min(tv)))
span_u = sorted(s[0] for s in spans)
span_v = sorted(s[1] for s in spans)
print(f"tri u-span: med={span_u[len(span_u)//2]:.4f} p90={span_u[int(len(span_u)*0.9)]:.4f} "
      f"max={span_u[-1]:.4f} (med u-step={med_du:.4f})")
print(f"tri v-span: med={span_v[len(span_v)//2]:.4f} p90={span_v[int(len(span_v)*0.9)]:.4f} "
      f"max={span_v[-1]:.4f} (med v-step={med_dv:.4f})")
n_mon_u = sum(1 for s in span_u if s > 4*med_du)
n_mon_v = sum(1 for s in span_v if s > 4*med_dv)
print(f"monster tris (u-span>4 steps): {n_mon_u}, (v-span>4 steps): {n_mon_v}")

# fan census: vertex degree among B and I
deg = defaultdict(int)
for t in tris:
    for i in t:
        deg[i] += 2
top = sorted(deg.items(), key=lambda kv: -kv[1])[:8]
print("top-degree verts:", [(f"{'B' if k < n_b else 'I'}{k}", v) for k, v in top])

# ── 5. Chain order: consecutive interior append-order geometry ────
if len(inter) > 2:
    steps = []
    for i in range(len(inter)-1):
        steps.append(((inter[i][0]-inter[i+1][0])**2 +
                      (inter[i][1]-inter[i+1][1])**2) ** 0.5)
    ss = sorted(steps)
    print(f"chain steps (append order): min={ss[0]:.4f} med={ss[len(ss)//2]:.4f} "
          f"p90={ss[int(len(ss)*0.9)]:.4f} max={ss[-1]:.4f}")
    long_jumps = sum(1 for s in ss if s > 4*max(med_du, med_dv))
    print(f"chain long jumps (>4 steps): {long_jumps}/{len(steps)}")
