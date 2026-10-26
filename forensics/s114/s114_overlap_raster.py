#!/usr/bin/env python3
"""s114: rasterize a band-lattice strip dump to localize double coverage."""
import sys

path = sys.argv[1]
pts = []
tris = []
nb = 0
for ln in open(path):
    w = ln.split()
    if not w:
        continue
    if w[0] == 'n_boundary':
        nb = int(w[1])
    elif w[0] == 'p':
        pts.append((float(w[1]), float(w[2])))
    elif w[0] == 't':
        tris.append((int(w[1]), int(w[2]), int(w[3])))

umin = min(p[0] for p in pts); umax = max(p[0] for p in pts)
vmin = min(p[1] for p in pts); vmax = max(p[1] for p in pts)
NU, NV = 400, 600
du = (umax - umin) / NU
dv = (vmax - vmin) / NV

def in_tri(px, py, t):
    (x1, y1), (x2, y2), (x3, y3) = pts[t[0]], pts[t[1]], pts[t[2]]
    d1 = (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)
    d2 = (x3 - x2) * (py - y2) - (y3 - y2) * (px - x2)
    d3 = (x1 - x3) * (py - y3) - (y1 - y3) * (px - x3)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)

def poly_inside(px, py):
    # ray casting on the boundary ring
    inside = False
    n = nb
    for i in range(n):
        a = pts[i]; b = pts[(i + 1) % n]
        if (a[1] > py) != (b[1] > py):
            xint = a[0] + (py - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if px < xint:
                inside = not inside
    return inside

# bbox of each triangle for speed
import collections
grid = collections.defaultdict(int)
for t in tris:
    xs = [pts[i][0] for i in t]; ys = [pts[i][1] for i in t]
    i0 = max(0, int((min(xs) - umin) / du) - 1)
    i1 = min(NU - 1, int((max(xs) - umin) / du) + 1)
    j0 = max(0, int((min(ys) - vmin) / dv) - 1)
    j1 = min(NV - 1, int((max(ys) - vmin) / dv) + 1)
    for j in range(j0, j1 + 1):
        py = vmin + (j + 0.5) * dv
        for i in range(i0, i1 + 1):
            px = umin + (i + 0.5) * du
            if in_tri(px, py, t):
                grid[(i, j)] += 1

n0 = n1 = n2 = n_out = 0
zones = collections.Counter()
for (i, j), c in grid.items():
    px = umin + (i + 0.5) * du
    py = vmin + (j + 0.5) * dv
    ins = poly_inside(px, py)
    if c == 0:
        continue
    if not ins:
        n_out += 1
    elif c == 1:
        n1 += 1
    elif c >= 2:
        n2 += 1
        zones[round(py, 1)] += 1
print(f"{path.split('/')[-1]}: cells cov1={n1} cov2+={n2} outside={n_out}")
print("  double-coverage v-zones:", sorted(zones.items())[:20])
