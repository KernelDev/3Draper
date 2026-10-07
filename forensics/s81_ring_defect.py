#!/usr/bin/env python3
"""s81: reconstruct the f3 end-plane ring (BREP#62542 face s52736,
842 ring pts / 840 fan tris) and find why earcutr's area differs from
the shoelace area by 2.4e-3 relative (self-intersection / spike /
duplicate scan).
"""
import math
from collections import defaultdict

OBJ = "/tmp/s81_faceobjs/brep62542_f3_s52736_Plane.obj"

verts = []
tris = []
with open(OBJ) as f:
    for line in f:
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:]
            tris.append(tuple(int(v) - 1 for v in (a, b, c)))

print(f"verts={len(verts)} tris={len(tris)}")

# 2D coords on the plane z=0.6 (BREP-local): project to (x, y)
P = [(v[0], v[1]) for v in verts]

# edge use counts
use = defaultdict(int)
for t in tris:
    for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
        use[(min(a, b), max(a, b))] += 1

boundary = [e for e, c in use.items() if c == 1]
interior_odd = [e for e, c in use.items() if c not in (1, 2)]
print(f"boundary edges: {len(boundary)}, non-1/2-use edges: {len(interior_odd)}")

# order the boundary loop(s)
adj = defaultdict(list)
for a, b in boundary:
    adj[a].append(b)
    adj[b].append(a)
deg_bad = [v for v, ns in adj.items() if len(ns) != 2]
print(f"boundary verts with deg!=2: {len(deg_bad)}: {deg_bad[:10]}")

# walk the loop from an arbitrary boundary vertex
ring = []
if boundary:
    start = boundary[0][0]
    prev, cur = None, start
    while True:
        ring.append(cur)
        nxts = [n for n in adj[cur] if n != prev]
        if not nxts:
            break
        nxt = nxts[0]
        if nxt == start:
            break
        prev, cur = cur, nxt
        if len(ring) > len(boundary) + 5:
            break
print(f"ring length: {len(ring)} (expected 842)")

# --- diagnostics on the ring ---
def shoelace(pts):
    s = 0.0
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        s += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
    return s

# 1. duplicate consecutive points
dups = sum(1 for i in range(len(ring)) if P[ring[i]] == P[ring[(i + 1) % len(ring)]])
print(f"consecutive duplicate pts: {dups}")

# 2. near-duplicate consecutive (dist < 1e-9, 1e-6, 1e-4)
for tol in (1e-9, 1e-6, 1e-4, 1e-3):
    n = sum(1 for i in range(len(ring))
            if math.dist(P[ring[i]], P[ring[(i + 1) % len(ring)]]) < tol)
    print(f"  consecutive dist<{tol}: {n}")

# 3. spikes: A->B->A backtracking (within eps)
spikes = 0
n = len(ring)
for i in range(n):
    a, b, c = ring[i], ring[(i + 1) % n], ring[(i + 2) % n]
    if math.dist(P[a], P[c]) < 1e-6 and math.dist(P[a], P[b]) > 1e-4:
        spikes += 1
print(f"spikes (A,B,A with |AB|>1e-4): {spikes}")

# 4. self-intersections: non-adjacent segment crossings (Bentley-Ottmann
#    too heavy; O(n^2) with bbox prefilter on 842 segments is fine)
def seg_int(p1, p2, p3, p4):
    d1 = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0])
    d2 = (p2[0]-p1[0])*(p4[1]-p1[1]) - (p2[1]-p1[1])*(p4[0]-p1[0])
    d3 = (p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])
    d4 = (p4[0]-p3[0])*(p2[1]-p3[1]) - (p4[1]-p3[1])*(p2[0]-p3[0])
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    return False

segs = []
for i in range(n):
    a, b = ring[i], ring[(i + 1) % n]
    x1, y1 = P[a]; x2, y2 = P[b]
    segs.append((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2), i))

crosses = []
for i in range(n):
    s1 = segs[i]
    for j in range(i + 2, n):
        if i == 0 and j == n - 1:
            continue
        s2 = segs[j]
        if s1[2] < s2[0] or s2[2] < s1[0] or s1[3] < s2[1] or s2[3] < s1[1]:
            continue
        a, b = P[ring[i]], P[ring[(i + 1) % n]]
        c, d = P[ring[j]], P[ring[(j + 1) % n]]
        if seg_int(a, b, c, d):
            crosses.append((i, j))
print(f"self-intersections: {len(crosses)}: {crosses[:12]}")

# 5. areas: shoelace(ring) vs fan-triangulation signed area
ring_area2 = shoelace([P[v] for v in ring])
tri_area2 = 0.0
for t in tris:
    a, b, c = P[t[0]], P[t[1]], P[t[2]]
    tri_area2 += (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
print(f"ring_area2={ring_area2:.6f} tri_area2={tri_area2:.6f} "
      f"rel={abs(tri_area2-ring_area2)/abs(ring_area2):.3e}")

# 6. winding sanity: signed area sign
print(f"ring winding: {'CCW' if ring_area2 > 0 else 'CW'}")

# 7. zero-area / degenerate triangles in the fan
degen = 0
for t in tris:
    a, b, c = P[t[0]], P[t[1]], P[t[2]]
    s = abs((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]))
    if s < 1e-12:
        degen += 1
print(f"degenerate fan tris (|area2|<1e-12): {degen}")

# 8. bbox + thinness in-plane
xs = [P[v][0] for v in ring]; zs = [P[v][1] for v in ring]
print(f"ring bbox: x=[{min(xs):.4f},{max(xs):.4f}] z=[{min(zs):.4f},{max(zs):.4f}]")
