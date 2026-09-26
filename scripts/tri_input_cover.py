#!/usr/bin/env python3
"""Session-58: polygon topology of the spike-chain ring.

Is the [rim + chain] ring simple (non-self-intersecting)? Shoelace area
vs rectangle area; segment crossings; closing-edge geometry; where the
chain attaches; row-structure of the triangulation.

Usage: tri_input_cover.py <dump.txt>
"""
import sys
from pathlib import Path

src = Path(sys.argv[1])
mode = None
bnd, inter, tris = [], [], []
for line in src.read_text().splitlines():
    f = line.split()
    if not f:
        continue
    if f[0] in ("type", "boundary", "interior", "tris"):
        if f[0] == "boundary":
            mode = "b"
        elif f[0] == "interior":
            mode = "i"
        elif f[0] == "tris":
            mode = "t"
        continue
    if mode == "b":
        bnd.append((float(f[1]), float(f[2])))
    elif mode == "i":
        inter.append((float(f[1]), float(f[2])))
    elif mode == "t":
        tris.append((int(f[1]), int(f[2]), int(f[3])))

n_b = len(bnd)
ring = bnd + inter  # the earcutr input ring: [boundary][interior chain]
N = len(ring)
print(f"ring: {n_b} rim + {len(inter)} chain = {N} pts")

# ring endpoints geometry
print(f"b0={bnd[0]} b{n_b-1}={bnd[n_b-1]}")
print(f"chain start I0={inter[0]}  chain end I{len(inter)-1}={inter[-1]}")

# shoelace
def shoelace(pts):
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i+1) % len(pts)]
        s += x1*y2 - x2*y1
    return s / 2

area = shoelace(ring)
us = [p[0] for p in bnd]
vs = [p[1] for p in bnd]
rect = (max(us)-min(us)) * (max(vs)-min(vs))
print(f"shoelace area={area:.5f} rect area={rect:.5f} ratio={area/rect:.4f}")
print(f"orientation: {'CCW' if area > 0 else 'CW'}")

# segment intersection test (proper crossings, excluding adjacent edges)
def seg_inter(p1, p2, p3, p4):
    def d(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    d1 = d(p3, p4, p1)
    d2 = d(p3, p4, p2)
    d3 = d(p1, p2, p3)
    d4 = d(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False

segs = [(ring[i], ring[(i+1) % N], i) for i in range(N)]
crossings = []
for i in range(N):
    for j in range(i+1, N):
        # skip adjacent
        if j == i or (i+1) % N == j or (j+1) % N == i:
            continue
        a1, a2, _ = segs[i]
        b1, b2, _ = segs[j]
        if seg_inter(a1, a2, b1, b2):
            crossings.append((i, j))
print(f"proper segment crossings: {len(crossings)}")
for c in crossings[:10]:
    i, j = c
    def desc(k):
        kind = "rim" if k < n_b else f"chain#{k-n_b}"
        a = ring[k]
        b = ring[(k+1) % N]
        return f"{kind} ({a[0]:.3f},{a[1]:.3f})->({b[0]:.3f},{b[1]:.3f})"
    print(f"  X {desc(i)}  ×  {desc(j)}")

# chain structure: rows (constant v runs) and jumps
rows = []
cur = [inter[0]]
for p in inter[1:]:
    if abs(p[1] - cur[-1][1]) < 1e-6:
        cur.append(p)
    else:
        rows.append(cur)
        cur = [p]
rows.append(cur)
print(f"chain rows: {len(rows)}; row lens: {[len(r) for r in rows[:5]]}...")
print(f"row v-order: {[round(r[0][1],3) for r in rows[:6]]} ...")
print(f"row u-directions: {['R' if r[0][0] < r[-1][0] else 'L' for r in rows[:6]]}")

# triangle UV overlap census (brute force, bbox prefilter) — count pairs
# whose UV bounding boxes overlap AND triangles actually intersect.
def tri_bb(t):
    xs = [ring[i][0] for i in t]
    ys = [ring[i][1] for i in t]
    return min(xs), max(xs), min(ys), max(ys)

def segs_of(t):
    return [(ring[t[k]], ring[t[(k+1) % 3]]) for k in range(3)]

def tri_area2(t):
    (x1, y1), (x2, y2), (x3, y3) = [ring[i] for i in t]
    return (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1)

def point_in_tri(p, t):
    # t CCW
    a, b, c = [ring[i] for i in t]
    def cross(o, u, v):
        return (u[0]-o[0])*(v[1]-o[1]) - (u[1]-o[1])*(v[0]-o[0])
    return cross(a, b, p) >= -1e-12 and cross(b, c, p) >= -1e-12 and cross(c, a, p) >= -1e-12

def tri_pair_overlap(t1, t2):
    # conservative: any segment properly crossing, or a vertex inside
    for p, q in segs_of(t1):
        for r, s in segs_of(t2):
            if seg_inter(p, q, r, s):
                return True
    for i in t1:
        if point_in_tri(ring[i], t2):
            return True
    for i in t2:
        if point_in_tri(ring[i], t1):
            return True
    return False

# normalize orientation CCW for the tests
ccw = []
for t in tris:
    if tri_area2(t) < 0:
        t = (t[0], t[2], t[1])
    ccw.append(t)

bbs = [tri_bb(t) for t in ccw]
n_overlap = 0
samples = []
for i in range(len(ccw)):
    x0, x1, y0, y1 = bbs[i]
    for j in range(i+1, len(ccw)):
        a0, a1, b0, b1 = bbs[j]
        if a0 > x1 or x0 > a1 or b0 > y1 or y0 > b1:
            continue
        if tri_pair_overlap(ccw[i], ccw[j]):
            n_overlap += 1
            if len(samples) < 5:
                samples.append((i, j))
print(f"UV overlapping tri pairs (bbox-prefiltered, conservative): {n_overlap}")
for i, j in samples:
    print(f"  pair {i},{j}: {[round(x,3) for x in ccw[i]]} vs {[round(x,3) for x in ccw[j]]}")
