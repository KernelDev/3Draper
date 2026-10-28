#!/usr/bin/env python3
"""s115: identify overlapping triangle PAIRS in a band-lattice strip
dump (the 0.65% retry double coverage) — which strips, which shapes,
drag-lines vs scales."""
import sys
import collections

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

# grid cell -> set of triangles covering it
grid = collections.defaultdict(set)
for ti, t in enumerate(tris):
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
                grid[(i, j)].add(ti)

# overlapping pairs
pairs = collections.Counter()
pair_cells = collections.Counter()
for cell, ts in grid.items():
    if len(ts) >= 2:
        tl = sorted(ts)
        for x in range(len(tl)):
            for y in range(x + 1, len(tl)):
                pairs[(tl[x], tl[y])] += 1
                pair_cells[(tl[x], tl[y])] = pair_cells[(tl[x], tl[y])]

print(f"overlapping pairs: {len(pairs)}")
# classify by triangle shapes: aspect (longest edge vs height)
def dims(t):
    (x1, y1), (x2, y2), (x3, y3) = pts[t[0]], pts[t[1]], pts[t[2]]
    a = ((x2-x1)**2 + (y2-y1)**2) ** 0.5
    b = ((x3-x2)**2 + (y3-y2)**2) ** 0.5
    c = ((x1-x3)**2 + (y1-y3)**2) ** 0.5
    s = (a + b + c) / 2
    area = max(s*(s-a)*(s-b)*(s-c), 0) ** 0.5
    longest = max(a, b, c)
    h = 2 * area / max(longest, 1e-30)  # height over longest edge
    return longest, h, area

rows = []
for (t1, t2), cnt in pairs.most_common():
    l1, h1, a1 = dims(tris[t1])
    l2, h2, a2 = dims(tris[t2])
    rows.append((cnt, t1, t2, l1, h1, a1, l2, h2, a2))

print("top-15 pairs by overlap cell count:")
print("cells  tri1  tri2  | t1_len    t1_h     t1_area   | t2_len    t2_h     t2_area")
for r in rows[:15]:
    print(f"{r[0]:5d} {r[1]:5d} {r[2]:5d} | {r[3]:.6f} {r[4]:.6f} {r[5]:.3e} | {r[6]:.6f} {r[7]:.6f} {r[8]:.3e}")

# aggregate: how many pairs involve a "drag-line" (len/h > 20)?
drag = sum(1 for r in rows if (r[3]/max(r[4],1e-30) > 20) or (r[6]/max(r[7],1e-30) > 20))
print(f"\npairs with a drag-line member (len/h>20): {drag} / {len(rows)}")
# distribution of overlap sizes
sizes = collections.Counter(min(r[0], 10) for r in rows)
print("overlap-size histogram (cells, capped 10):", sorted(sizes.items()))
# v-range of overlaps
vzs = collections.Counter()
for (t1, t2), cnt in pairs.items():
    vs = [pts[i][1] for i in tris[t1]] + [pts[i][1] for i in tris[t2]]
    vzs[round((min(vs)+max(vs))/2, 1)] += cnt
print("overlap cells by mid-v zone:", sorted(vzs.items()))
