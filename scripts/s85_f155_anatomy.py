#!/usr/bin/env python3
"""Session-85: anatomy of the (153,155) SLEEVE Cone x Plane fold family.

All 25 REAL pairs: Plane f153 strip tris [17XX, 12YY, 17XX+1] vs Cone f155
fan tris [429|431, 17XX, 17XX+1] across the shared circle (r~0.64, z~-1.53),
snAng=135, mesh flattens to ~178. Cone-side self-distance up to 0.196 ->
the fan tris are NOT on the cone surface.

Questions:
  1. Where are apexes 429/431? On the shared-circle plane? On the cone axis?
  2. Is the whole f155 a 2-apex fan, or is the fan a local defect?
  3. What does the rest of f155 (non-fan) look like — the far rim ring?
  4. Fit the true cone (axis, half-angle) from far-rim verts; measure each
     f155 tri centroid distance to the cone surface.
  5. f153: plane fit, ring structure, is the strip healthy?
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

d = Path(__file__).resolve().parent.parent / "s85_out"
objp = d / "brep2_SHAFT_SLEEVE (BREP#32629).obj"
fmapp = d / "brep2_SHAFT_SLEEVE (BREP#32629).fmap"

verts = []
tris = []
for line in objp.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))

fid_of_tri = {}
for line in fmapp.read_text().splitlines():
    parts = line.split()
    if parts[0] == "t":
        fid_of_tri[int(parts[1])] = int(parts[2])

# probe vertex ids are 0-indexed into the same vertex array
tris_of_face = defaultdict(list)
for ti, fid in fid_of_tri.items():
    tris_of_face[fid].append(ti)

print(f"total verts={len(verts)} tris={len(tris)}")
for fid in (153, 155):
    ts = tris_of_face.get(fid, [])
    print(f"face {fid}: {len(ts)} tris")

A = 153  # Plane
B = 155  # Cone

# --- shared circle: ring verts used by both faces near z=-1.53
va = set()
for ti in tris_of_face[A]:
    va.update(tris[ti])
vb = set()
for ti in tris_of_face[B]:
    vb.update(tris[ti])
shared = va & vb
print(f"\nface A verts={len(va)}  face B verts={len(vb)}  shared={len(shared)}")
zs = defaultdict(list)
for v in shared:
    zs[round(verts[v][2], 4)].append(v)
print("shared-vert z histogram:", {k: len(v) for k, v in sorted(zs.items())})

# --- apex positions
for ap in (429, 431):
    print(f"apex {ap}: {verts[ap]}  (in A={ap in va}, in B={ap in vb})")

# --- f155 fan structure
fan = [ti for ti in tris_of_face[B] if 429 in tris[ti] or 431 in tris[ti]]
print(f"\nf155: fan tris (429/431) = {len(fan)} / {len(tris_of_face[B])}")
deg = defaultdict(int)
for ti in tris_of_face[B]:
    for v in tris[ti]:
        deg[v] += 1
top = sorted(deg.items(), key=lambda kv: -kv[1])[:8]
print("f155 top-degree verts:", top)

# f155 verts not in the fan: the far rim?
nonfan = [ti for ti in tris_of_face[B] if ti not in set(fan)]
nfverts = set()
for ti in nonfan:
    nfverts.update(tris[ti])
print(f"f155 non-fan tris={len(nonfan)}, their verts={len(nfverts)}")
if nfverts:
    zvals = sorted({round(verts[v][2], 3) for v in nfverts})
    print("non-fan vert z values:", zvals[:10], "..." if len(zvals) > 10 else "")

# --- ring ring: the 17XX and 12YY families in face A
ringA = sorted(v for v in va if v not in vb)
print(f"\nface-A-only verts: {len(ringA)}  range {min(ringA)}..{max(ringA)}")
inner = sorted(v for v in va if 1282 <= v <= 1332)
ringX = sorted(v for v in shared)
print(f"shared ring verts ({len(ringX)}): {ringX}")
print(f"inner 12XX verts in A ({len(inner)}): {inner[:5]}..{inner[-5:]}")

# geometry of the shared ring (circle fit)
cxs = [verts[v][0] for v in ringX]
cys = [verts[v][1] for v in ringX]
czs = [verts[v][2] for v in ringX]
cx, cy = sum(cxs) / len(cxs), sum(cys) / len(cys)
radii = [math.hypot(verts[v][0] - cx, verts[v][1] - cy) for v in ringX]
print(f"\nshared ring: center=({cx:.4f},{cy:.4f}) z={sum(czs)/len(czs):.4f} "
      f"r min/max={min(radii):.4f}/{max(radii):.4f}")

# --- cone fit from non-fan verts (far rim) if available
def cone_report(label, pts):
    if not pts:
        print(f"{label}: no points")
        return
    zs_ = [p[2] for p in pts]
    rs_ = [math.hypot(p[0] - cx, p[1] - cy) for p in pts]
    print(f"{label}: n={len(pts)} z=[{min(zs_):.4f},{max(zs_):.4f}] "
          f"r=[{min(rs_):.4f},{max(rs_):.4f}]")

cone_report("non-fan(far rim?)", [verts[v] for v in sorted(nfverts)])

# --- distance of fan tri centroids to the line/cone through apex & ring
# ruled-surface check: every fan tri has apex + 2 ring verts. If the apex
# were the cone tip, tri lies exactly on the ruled cone. Measure centroid
# distance to the surface spanned by apex + ring circle (cone with tip at
# apex through the ring circle) — for each tri use its own apex.
def dist_point_cone(p, apex, cx, cy, zr, r_ring):
    # cone: tip=apex, passes through circle (cx,cy,zr,r_ring).
    # axis = apex->circle-center. half-angle from r_ring and height.
    ax, ay, az = apex
    h = zr - az  # signed along axis (assume axis ~ z)
    if abs(h) < 1e-9:
        return None
    tan_half = r_ring / abs(h)
    # vector from apex to point
    vx, vy, vz = p[0] - ax, p[1] - ay, p[2] - az
    # axial component (axis ~ (0,0,sign(h)))
    ax_comp = vz * (1 if h > 0 else -1)
    radial = math.hypot(p[0] - cx - 0, p[1] - cy - 0)
    # apex-to-point horizontal offset (apex may be off-center)
    return radial, ax_comp, tan_half

bad = 0
samples = []
for ti in fan:
    t = tris[ti]
    apex = 429 if 429 in t else 431
    others = [v for v in t if v != apex]
    cen = tuple(sum(verts[v][k] for v in t) / 3 for k in range(3))
    # ring radius from the two ring verts
    rr = sum(math.hypot(verts[v][0] - cx, verts[v][1] - cy) for v in others) / 2
    zr = sum(verts[v][2] for v in others) / 2
    radial, ax_comp, tan_half = dist_point_cone(cen, verts[apex], cx, cy, zr, rr)
    expect = ax_comp * tan_half
    dev = radial - expect
    samples.append((ti, apex, radial, ax_comp, dev))
    if abs(dev) > 0.02:
        bad += 1
print(f"\nfan tris off their own apex-ruled-cone by >0.02: {bad}/{len(fan)}")
for s in samples[:8]:
    print(f"  tri {s[0]} apex {s[1]} radial={s[2]:.4f} axial={s[3]:.4f} dev={s[4]:+.4f}")

# --- is the fan flat in the ring plane? normal of fan tris vs z-axis
def normal(t):
    (ax_, ay_, az_), (bx, by, bz), (cxx, cyy, czz) = (verts[t[0]], verts[t[1]], verts[t[2]])
    ux, uy, uz = bx - ax_, by - ay_, bz - az_
    vx, vy, vz = cxx - ax_, cyy - ay_, czz - az_
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    n = math.sqrt(nx * nx + ny * ny + nz * nz)
    return nx / n, ny / n, nz / n

nz_flat = sum(1 for ti in fan if abs(normal(tris[ti])[2]) > 0.9)
print(f"fan tris with |nz|>0.9 (flat/horizontal): {nz_flat}/{len(fan)}")

# --- face A plane fit
zsA = [verts[v][2] for v in va]
print(f"\nface A z range: [{min(zsA):.4f},{max(zsA):.4f}] (planar-xy if narrow)")

# apex distance to ring plane and to cone axis
for ap in (429, 431):
    x, y, z = verts[ap]
    print(f"apex {ap}: dz_to_ring_plane={z - sum(czs)/len(czs):+.4f} "
          f"r_from_axis={math.hypot(x - cx, y - cy):.4f}")
