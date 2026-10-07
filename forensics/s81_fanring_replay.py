#!/usr/bin/env python3
"""s81: replay the FAN_GUARD area contract on the dumped UV ring of
brep62542_f3_Plane (m=842) — find WHERE the 2.4e-3 earcutr-vs-shoelace
area mismatch lives (self-intersection / pinch / wrap in UV).
"""
import math

SRC = "/tmp/s81_fanring/brep62542_f3_Plane.txt"

pts = []
tris = []
meta = {}
with open(SRC) as f:
    for line in f:
        if line.startswith("m="):
            meta = dict(kv.split("=") for kv in line.split())
        elif line.startswith("p "):
            _, u, v = line.split()
            pts.append((float(u), float(v)))
        elif line.startswith("t "):
            _, a, b, c = line.split()
            tris.append((int(a), int(b), int(c)))

m = len(pts)
print(f"meta={meta} pts={m} fan_tris={len(tris)}")

def shoelace(ring):
    s = 0.0
    n = len(ring)
    for i in range(n):
        j = (i + 1) % n
        s += ring[i][0] * ring[j][1] - ring[j][0] * ring[i][1]
    return s

ring_area2 = shoelace(pts)
print(f"ring_area2={ring_area2:.9f}")

# fan tri signed area (as the guard computes it)
tri_area2 = 0.0
for t in tris:
    a, b, c = pts[t[0]], pts[t[1]], pts[t[2]]
    tri_area2 += (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
print(f"fan_tri_area2={tri_area2:.9f} rel_to_ring={abs(tri_area2-ring_area2)/abs(ring_area2):.3e}")

# UV bbox
us = [p[0] for p in pts]; vs = [p[1] for p in pts]
print(f"u range [{min(us):.6f},{max(us):.6f}] span={max(us)-min(us):.6f}")
print(f"v range [{min(vs):.6f},{max(vs):.6f}] span={max(vs)-min(vs):.6f}")

# consecutive dup / near-dup in UV
for tol in (1e-9, 1e-7, 1e-5, 1e-3):
    n = sum(1 for i in range(m) if math.dist(pts[i], pts[(i+1)%m]) < tol)
    print(f"consecutive UV dist<{tol}: {n}")

# edge length distribution
els = sorted(math.dist(pts[i], pts[(i+1)%m]) for i in range(m))
print(f"edge len: min={els[0]:.3e} p5={els[m//20]:.3e} med={els[m//2]:.4f} max={els[-1]:.4f}")

# self-intersection scan (O(n^2) bbox-prefiltered)
def seg_int(p1, p2, p3, p4):
    d1 = (p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0])
    d2 = (p2[0]-p1[0])*(p4[1]-p1[1]) - (p2[1]-p1[1])*(p4[0]-p1[0])
    d3 = (p4[0]-p3[0])*(p1[1]-p3[1]) - (p4[1]-p3[1])*(p1[0]-p3[0])
    d4 = (p4[0]-p3[0])*(p2[1]-p3[1]) - (p4[1]-p3[1])*(p2[0]-p3[0])
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))

segs = []
for i in range(m):
    a, b = pts[i], pts[(i+1)%m]
    segs.append((min(a[0],b[0]), min(a[1],b[1]), max(a[0],b[0]), max(a[1],b[1]), i))
crosses = []
for i in range(m):
    s1 = segs[i]
    for j in range(i+2, m):
        if i == 0 and j == m-1:
            continue
        s2 = segs[j]
        if s1[2] < s2[0] or s2[2] < s1[0] or s1[3] < s2[1] or s2[3] < s1[1]:
            continue
        a, b = pts[i], pts[(i+1)%m]
        c, d = pts[j], pts[(j+1)%m]
        if seg_int(a, b, c, d):
            crosses.append((i, j))
print(f"UV self-intersections: {len(crosses)}")
for i, j in crosses[:10]:
    a, b = pts[i], pts[(i+1)%m]
    c, d = pts[j], pts[(j+1)%m]
    print(f"  seg#{i} {a}->{b} X seg#{j} {c}->{d}")

# pinch: near-coincident NON-consecutive vertices
pinches = []
for i in range(m):
    for j in range(i+2, m):
        if i == 0 and j == m-1:
            continue
        if math.dist(pts[i], pts[j]) < 1e-6:
            pinches.append((i, j, math.dist(pts[i], pts[j])))
print(f"pinches (non-adjacent dist<1e-6): {len(pinches)}: {pinches[:8]}")

# winding defects: local turn analysis — total turning should be ±2pi
def turn(p0, p1, p2):
    v1 = (p1[0]-p0[0], p1[1]-p0[1])
    v2 = (p2[0]-p1[0], p2[1]-p1[1])
    cross = v1[0]*v2[1] - v1[1]*v2[0]
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    return math.atan2(cross, dot)

total_turn = 0.0
worry = []
for i in range(m):
    t = turn(pts[(i-1)%m], pts[i], pts[(i+1)%m])
    total_turn += t
    if abs(t) > math.pi/2:
        worry.append((i, round(math.degrees(t), 1)))
print(f"total turning = {math.degrees(total_turn):.1f} deg (simple => ±360)")
print(f"sharp turns >90deg: {len(worry)}: {worry[:12]}")
