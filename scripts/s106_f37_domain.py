#!/usr/bin/env python3
"""s106: visualize face-37 #39570 UV domain + grid + rejected points + fan."""
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

fm.fontManager.addfont('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']

DUMP = "/home/z/my-project/3Draper/forensics/s106/uvaudit/uvaudit_step39570.txt"

poly, holes, chords = [], [], []
cur_hole = None
for line in open(DUMP):
    m = re.match(r"POLY\s+(\d+) u=\s*([\d.-]+) v=\s*([\d.-]+)", line)
    if m:
        poly.append((float(m.group(2)), float(m.group(3))))
        continue
    m = re.match(r"HOLE (\d+) n=(\d+)", line)
    if m:
        cur_hole = []
        holes.append(cur_hole)
        continue
    m = re.match(r"\s+HP\s+\d+ u=\s*([\d.-]+) v=\s*([\d.-]+)", line)
    if m and cur_hole is not None:
        cur_hole.append((float(m.group(1)), float(m.group(2))))
        continue
    m = re.match(r"CHORD tri#(\d+) verts=\((\d+),(\d+),(\d+)\) poly=\((\d+),(\d+),(\d+)\)", line)
    if m:
        chords.append((int(m.group(5)), int(m.group(6)), int(m.group(7))))

# Reconstruct the grid (n_u=12, n_v=20, u [1.5708,4.7124], v [0.3127,5.9500])
u_min, u_max, v_min, v_max = 1.5708, 4.7124, 0.3127, 5.9500
n_u, n_v = 12, 20
u_span, v_span = u_max - u_min, v_max - v_min

# Point-in-polygon (ray cast) + on-hole check approximating the filter
def inside(pt, ring):
    x, y = pt
    n, c = len(ring), False
    for i in range(n):
        x1, y1 = ring[i]
        x2, y2 = ring[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-30) + x1:
            c = not c
    return c

def dist_to_seg(p, a, b):
    px, py = p; ax, ay = a; bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0 if L2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L2))
    qx, qy = ax + t * dx, ay + t * dy
    return ((px - qx) ** 2 + (py - qy) ** 2) ** 0.5

def on_ring(p, ring, tol):
    n = len(ring)
    return any(dist_to_seg(p, ring[i], ring[(i + 1) % n]) < tol for i in range(n))

span_max = max(u_span, v_span)
btol = max(span_max * 1e-6, 1e-9)
# NOTE: real filter uses a cached grid + is_point_on_boundary; here tol is
# exaggerated to 0.02 to visualize "near-boundary" rejections too.
vis_tol = 0.02

kept, rej_domain, rej_bnd = [], [], []
for j in range(1, n_v):
    v = v_min + v_span * j / n_v
    for i in range(1, n_u):
        u = u_min + u_span * i / n_u
        p = (u, v)
        if not inside(p, poly):
            rej_domain.append(p)
        elif on_ring(p, poly, vis_tol) or any(on_ring(p, h, vis_tol) for h in holes):
            rej_bnd.append(p)
        else:
            kept.append(p)

fig, ax = plt.subplots(figsize=(11, 9), constrained_layout=True)
if len(poly) > 2:
    xs = [p[0] for p in poly] + [poly[0][0]]
    ys = [p[1] for p in poly] + [poly[0][1]]
    ax.plot(xs, ys, "-", color="#1a6faa", lw=1.2, label="outer bnd")
for hi, h in enumerate(holes):
    if len(h) > 2:
        xs = [p[0] for p in h] + [h[0][0]]
        ys = [p[1] for p in h] + [h[0][1]]
        ax.plot(xs, ys, "-", color="#c96a1a", lw=0.9, label="holes" if hi == 0 else None)
if kept:
    ax.scatter([p[0] for p in kept], [p[1] for p in kept], s=9, c="#1a9a4a", label=f"grid kept ({len(kept)})")
if rej_domain:
    ax.scatter([p[0] for p in rej_domain], [p[1] for p in rej_domain], s=9, c="#cc2222", marker="x",
               label=f"rej domain ({len(rej_domain)})")
if rej_bnd:
    ax.scatter([p[0] for p in rej_bnd], [p[1] for p in rej_bnd], s=9, c="#9966cc", marker="+",
               label=f"rej near-bnd tol=0.02 ({len(rej_bnd)})")
# fan triangles
for t in chords:
    tri = [poly[k] for k in t]
    xs = [p[0] for p in tri] + [tri[0][0]]
    ys = [p[1] for p in tri] + [tri[0][1]]
    ax.plot(xs, ys, "-", color="#000000", lw=0.35, alpha=0.55)
ax.set_xlabel("u"); ax.set_ylabel("v")
ax.set_title("face-37 #39570 (drill HOUSING): UV domain, cyl grid n_u=12 n_v=20, CHORD fans")
ax.legend(loc="upper left", fontsize=8)
ax.set_aspect("equal")
fig.savefig("/home/z/my-project/3Draper/forensics/s106/f37_domain.png", dpi=110)
print(f"poly={len(poly)} holes={len(holes)} chords={len(chords)}")
print(f"grid kept={len(kept)} rej_domain={len(rej_domain)} rej_bnd={len(rej_bnd)}")
print("saved forensics/s106/f37_domain.png")
