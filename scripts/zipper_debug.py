#!/usr/bin/env python3
"""Session-58: debug the grid+zipper construction on a real dump.

Reimplements try_grid_band_triangulate's grid + angular zipper in
Python and checks coverage area, so merge-rule bugs are found without
100s Rust rebuilds.

Usage: zipper_debug.py <dump.txt>
"""
import math
import sys
from pathlib import Path

src = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/tri_in/tri_0005_Torus.txt")
import sys

mode = None
bnd, inter = [], []
for line in src.read_text().splitlines():
    f = line.split()
    if not f:
        continue
    if f[0] == "boundary":
        mode = "b"
        continue
    if f[0] == "interior":
        mode = "i"
        continue
    if f[0] == "tris":
        mode = None
        continue
    if mode == "b":
        bnd.append((float(f[1]), float(f[2])))
    elif mode == "i":
        inter.append((float(f[1]), float(f[2])))

n_b = len(bnd)
print(f"rim={n_b} interior={len(inter)}")

# cluster
def cluster(vals):
    s = sorted(vals)
    tol = max((s[-1]-s[0]) * 1e-9, 1e-12)
    out = [s[0]]
    for x in s[1:]:
        if x - out[-1] > tol:
            out.append(x)
    return out

us = cluster([p[0] for p in inter])
vs = cluster([p[1] for p in inter])
n_u, n_v = len(us), len(vs)
print(f"lat {n_u}x{n_v} full={n_u*n_v == len(inter)}")
rect_area = (us[-1]-us[0]) * (vs[-1]-vs[0])

# ring area (shoelace)
ring_area2 = 0.0
for i in range(n_b):
    a, b = bnd[i], bnd[(i+1) % n_b]
    ring_area2 += a[0]*b[1] - b[0]*a[1]
ring_area = ring_area2 / 2
print(f"ring area={ring_area:.5f} rect area={rect_area:.5f} expect band={ring_area-rect_area:.5f}")

# grid triangles
grid_area = 0.0
for r in range(n_v-1):
    for c in range(n_u-1):
        # cell corners in UV (cluster values — the actual points coincide)
        p00 = (us[c], vs[r]); p10 = (us[c+1], vs[r])
        p11 = (us[c+1], vs[r+1]); p01 = (us[c], vs[r+1])
        for tri in ((p00, p10, p11), (p00, p11, p01)):
            cr = (tri[1][0]-tri[0][0])*(tri[2][1]-tri[0][1]) - (tri[1][1]-tri[0][1])*(tri[2][0]-tri[0][0])
            grid_area += cr / 2
print(f"grid area={grid_area:.5f} (expect {rect_area:.5f})")

# perimeter ring (UV coords)
perim = []
for c in range(n_u):
    perim.append((us[c], vs[0]))
for r in range(1, n_v):
    perim.append((us[n_u-1], vs[r]))
for c in range(n_u-2, -1, -1):
    perim.append((us[c], vs[n_v-1]))
for r in range(n_v-2, 0, -1):
    perim.append((us[0], vs[r]))
n_p = len(perim)
print(f"perimeter pts={n_p}")

o_u = (us[0] + us[-1]) / 2
o_v = (vs[0] + vs[-1]) / 2
def angle(p):
    return math.atan2(p[1]-o_v, p[0]-o_u)

def start_min(pts):
    best, best_a = 0, 1e18
    for i, p in enumerate(pts):
        a = angle(p)
        if a < best_a:
            best_a, best = a, i
    return best

rim0 = start_min(bnd)
rim_idx = [(rim0+k) % n_b for k in range(n_b)]
per0 = start_min(perim)
per_idx = [(per0+k) % n_p for k in range(n_p)]

rim_ang_raw = [angle(bnd[i]) for i in rim_idx]
per_ang_raw = [angle(perim[k]) for k in per_idx]

def unwrap(raw):
    out = list(raw)
    for k in range(1, len(out)):
        while out[k] < out[k-1]:
            out[k] += 2*math.pi
    return out

rim_ang = unwrap(rim_ang_raw)
per_ang = unwrap(per_ang_raw)
print(f"rim_ang[0]={rim_ang[0]:.3f} rim_ang[-1]={rim_ang[-1]:.3f} (+2pi start={rim_ang[0]+2*math.pi:.3f})")
print(f"per_ang[0]={per_ang[0]:.3f} per_ang[-1]={per_ang[-1]:.3f}")
mono = all(rim_ang[k] <= rim_ang[k+1] for k in range(n_b-1))
print(f"rim_ang monotone: {mono}")

# merge
def next_rim(i):
    return rim_ang[i+1] if i+1 < n_b else rim_ang[0] + 2*math.pi
def next_per(j):
    return per_ang[j+1] if j+1 < n_p else per_ang[0] + 2*math.pi

i = j = 0
band_tris = []  # (rim_i, rim_next, per_j) or (per_j, rim_i, per_next) — UV coords
while i < n_b or j < n_p:
    if i >= n_b:
        adv_rim = False
    elif j >= n_p:
        adv_rim = True
    else:
        adv_rim = next_rim(i) <= next_per(j)
    if adv_rim:
        band_tris.append((bnd[rim_idx[i]], bnd[rim_idx[(i+1) % n_b]], perim[per_idx[j % n_p]]))
        i += 1
    else:
        band_tris.append((perim[per_idx[j]], bnd[rim_idx[i % n_b]], perim[per_idx[(j+1) % n_p]]))
        j += 1

print(f"band tris={len(band_tris)} (expect {n_b + n_p})")

band_area = 0.0
for (a, b, c) in band_tris:
    cr = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    band_area += cr  # signed, /2 later
print(f"band signed area={band_area/2:.5f} |abs|={abs(band_area)/2:.5f} (expect {ring_area-rect_area:.5f})")
print(f"total = grid {grid_area:.5f} + band {band_area/2:.5f} = {grid_area + band_area/2:.5f} vs ring {ring_area:.5f}")

# find worst band triangles: negative or huge
bad = []
for k, (a, b, c) in enumerate(band_tris):
    cr = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    bad.append((cr, k, a, b, c))
bad.sort(key=lambda t: -abs(t[0]))
print("worst band tris by |area2|:")
for cr, k, a, b, c in bad[:6]:
    print(f"  #{k} area2={cr:.4f} a={tuple(round(x,3) for x in a)} b={tuple(round(x,3) for x in b)} c={tuple(round(x,3) for x in c)}")
