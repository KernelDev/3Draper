#!/usr/bin/env python3
"""Session-58: census of faces eligible for the grid+zipper path.

Eligibility (V1): ring CONVEX (one cross-sign, collinear allowed),
no holes, interior lattice FULL-RECT (u-clusters × v-clusters == n),
≥2 cols and ≥2 rows.

Usage: grid_band_census.py <dump_dir>
"""
import sys
from pathlib import Path

d = Path(sys.argv[1])


def load(p):
    mode = None
    bnd, holes, inter, tris = [], [], [], []
    cur_hole = None
    hdr = ""
    for line in p.read_text().splitlines():
        f = line.split()
        if not f:
            continue
        if f[0] == "type":
            hdr = line
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
            holes.append((float(f[1]), float(f[2])))
        elif mode == "i":
            inter.append((float(f[1]), float(f[2])))
        elif mode == "t":
            tris.append(1)
    return hdr, bnd, holes, inter, len(tris)


def cross2(o, a, b):
    return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])


def cluster(vals, tol=1e-9):
    s = sorted(vals)
    out = [[s[0]]]
    for x in s[1:]:
        if x - out[-1][-1] > tol:
            out.append([x])
        else:
            out[-1].append(x)
    return out


n_elig = 0
n_total = 0
for p in sorted(d.glob("tri_*.txt")):
    hdr, bnd, holes, inter, ntris = load(p)
    if not inter or not bnd:
        continue
    n_total += 1
    label = hdr.split("label=")[1] if "label=" in hdr else p.name
    n_b = len(bnd)
    pos = neg = 0
    for i in range(n_b):
        c = cross2(bnd[i], bnd[(i+1) % n_b], bnd[(i+2) % n_b])
        if c > 1e-12:
            pos += 1
        elif c < -1e-12:
            neg += 1
    convex = pos == 0 or neg == 0
    uc = cluster([p[0] for p in inter])
    vc = cluster([p[1] for p in inter])
    full = len(uc) * len(vc) == len(inter) and len(uc) >= 2 and len(vc) >= 2
    if convex and not holes and full:
        n_elig += 1
        print(f"ELIGIBLE {label}: ring={n_b} lat={len(uc)}x{len(vc)} tris={ntris} "
              f"{'CCW' if pos else 'CW'}")

print(f"\ntotal faces with interior: {n_total}, eligible: {n_elig}")
