#!/usr/bin/env python3
"""s81: UV-domain analysis of brep62542_f26 (Nurbs 4x10 fillet):
boundary polygon quality (self-intersection, wrap), interior Steiner
placement, result triangle overlap in UV, dropped-region evidence
(Euler: 220 bnd + 341 int -> expected ~900 tris, got 559).
"""
import math
import re
from collections import defaultdict

SRC = "/tmp/s81_tri/tri_0649_Nurbs.txt"

hdr = {}
bnd = []
interior = []
tris = []
section = None
with open(SRC) as f:
    for line in f:
        if line.startswith("type="):
            for key in ("type", "forward", "n_boundary", "n_holes", "n_interior", "n_tris"):
                m = re.search(rf"\b{key}=([^\s]+)", line)
                if m:
                    hdr[key] = m.group(1)
        elif line.startswith("boundary"):
            section = "b"
        elif line.startswith("interior"):
            section = "i"
        elif line.startswith("t "):
            _, a, b, c = line.split()
            tris.append((int(a), int(b), int(c)))
        elif line.startswith("b ") and section == "b":
            _, u, v = line.split()
            bnd.append((float(u), float(v)))
        elif line.startswith("i ") and section == "i":
            _, u, v = line.split()
            interior.append((float(u), float(v)))

print(f"hdr={hdr}")
print(f"boundary={len(bnd)} interior={len(interior)} tris={len(tris)}")

n_b = len(bnd)
V = n_b + len(interior)
print(f"Euler check: V={V} B={n_b} -> expected T={2*V-2-n_b}, got {len(tris)} (dropped {2*V-2-n_b-len(tris)})")

# bbox
us = [p[0] for p in bnd]; vs = [p[1] for p in bnd]
print(f"bnd u range [{min(us):.4f},{max(us):.4f}] v range [{min(vs):.4f},{max(vs):.4f}]")
ius = [p[0] for p in interior]; ivs = [p[1] for p in interior]
print(f"int u range [{min(ius):.4f},{max(ius):.4f}] v range [{min(ivs):.4f},{max(ivs):.4f}]")

# boundary self-intersection
def seg_int(p1, p2, p3, p4):
    d1 = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0])
    d2 = (p2[0]-p1[0])*(p4[1]-p1[1]) - (p2[1]-p1[1])*(p4[0]-p1[0])
    d3 = (p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])
    d4 = (p4[0]-p3[0])*(p2[1]-p3[1]) - (p4[1]-p3[1])*(p2[0]-p3[0])
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))

segs = []
for i in range(n_b):
    a, b = bnd[i], bnd[(i+1)%n_b]
    segs.append((min(a[0],b[0]), min(a[1],b[1]), max(a[0],b[0]), max(a[1],b[1]), i))
crosses = []
for i in range(n_b):
    s1 = segs[i]
    for j in range(i+2, n_b):
        if i == 0 and j == n_b-1:
            continue
        s2 = segs[j]
        if s1[2] < s2[0] or s2[2] < s1[0] or s1[3] < s2[1] or s2[3] < s1[1]:
            continue
        if seg_int(bnd[i], bnd[(i+1)%n_b], bnd[j], bnd[(j+1)%n_b]):
            crosses.append((i, j))
print(f"boundary self-intersections: {len(crosses)}: {crosses[:8]}")

# wrap detection: u jumps (periodic wrap in u)
jumps = []
for i in range(n_b):
    du = abs(bnd[i][0] - bnd[(i+1)%n_b][0])
    dv = abs(bnd[i][1] - bnd[(i+1)%n_b][1])
    span_u = max(us)-min(us)
    span_v = max(vs)-min(vs)
    if du > 0.5*span_u or dv > 0.5*span_v:
        jumps.append((i, bnd[i], bnd[(i+1)%n_b]))
print(f"half-span jumps (wrap candidates): {len(jumps)}: {jumps[:5]}")

# total turning
def turn(p0, p1, p2):
    v1 = (p1[0]-p0[0], p1[1]-p0[1])
    v2 = (p2[0]-p1[0], p2[1]-p1[1])
    return math.atan2(v1[0]*v2[1]-v1[1]*v2[0], v1[0]*v2[0]+v1[1]*v2[1])

total = sum(turn(bnd[(i-1)%n_b], bnd[i], bnd[(i+1)%n_b]) for i in range(n_b))
print(f"total turning: {math.degrees(total):.1f} deg")

# result triangle UV areas: negatives / zeros / overlaps
allpts = bnd + interior
neg = zero = 0
areas = []
for t in tris:
    a, b, c = allpts[t[0]], allpts[t[1]], allpts[t[2]]
    s = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    if s < 0:
        neg += 1
    if s == 0:
        zero += 1
    areas.append(abs(s)*0.5)
print(f"result tris: neg={neg} zero={zero} | min={min(areas):.3e} med={sorted(areas)[len(areas)//2]:.3e} max={max(areas):.3e}")

# vertex usage: unused points
used = set()
for t in tris:
    used.update(t)
print(f"unused points: {V - len(used)}/{V} (bnd unused: {n_b - sum(1 for i in range(n_b) if i in used)}, int unused: {len(interior) - sum(1 for i in range(n_b, V) if i in used)})")

# rim edge coverage
edges = set()
for t in tris:
    for k in range(3):
        a, b = t[k], t[(k+1)%3]
        edges.add((min(a, b), max(a, b)))
missing = sum(1 for i in range(n_b) if (min(i, (i+1)%n_b), max(i, (i+1)%n_b)) not in edges)
print(f"missing rim edges: {missing}/{n_b}")

# UV overlap: sum of |areas| vs polygon area (shoelace)
ring2 = 0.0
for i in range(n_b):
    j = (i+1)%n_b
    ring2 += bnd[i][0]*bnd[j][1] - bnd[j][0]*bnd[i][1]
poly_area = abs(ring2)*0.5
tri_sum = sum(areas)
print(f"polygon area={poly_area:.6f} sum|tri|={tri_sum:.6f} overlap_ratio={(tri_sum-poly_area)/max(poly_area,1e-30):.4f}")
