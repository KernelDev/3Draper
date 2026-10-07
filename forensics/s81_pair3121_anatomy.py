#!/usr/bin/env python3
"""s81: anatomy of the (3,121) Plane x Nurbs fold family in HOUSING_MIRROR
(BREP#62542): two needle fans on the end-plane (apex 368: 22 pairs, apex
393: 10 pairs) folding 180 deg against near-degenerate rim slivers of the
Nurbs fillet face 121 (cps=7x13, step 57293).

Questions:
  A. geometry of the two plane fans (apex pos, fan degree, triangle heights,
     rim chain length, base-edge lengths)
  B. the Nurbs-side slivers (heights, areas, which rim row)
  C. why PLANAR_FAN_GUARD (s79) did not trigger: domain m, fan degree,
     thinness of the plane domain
  D. is the shared rim edge chain consistent plane<->nurbs (point counts)?
"""
import math
from collections import defaultdict

OBJ = "/tmp/s81_final/brep4_HOUSING_MIRROR (BREP#62542).obj"
FMAP = "/tmp/s81_final/brep4_HOUSING_MIRROR (BREP#62542).fmap"

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

fid_by_tri = {}
with open(FMAP) as f:
    for line in f:
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_by_tri[int(ti)] = int(fid)
fids = [fid_by_tri.get(i, -1) for i in range(len(tris))]

print(f"verts={len(verts)} tris={len(tris)} fids={len(fids)}")

def dist(a, b):
    return math.dist(verts[a], verts[b])

def tri_area(a, b, c):
    ax, ay, az = verts[a]; bx, by, bz = verts[b]; cx, cy, cz = verts[c]
    ux, uy, uz = bx-ax, by-ay, bz-az
    vx, vy, vz = cx-ax, cy-ay, cz-az
    cxv = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)
    return 0.5*math.sqrt(cxv[0]**2+cxv[1]**2+cxv[2]**2)

# --- fan apexes from the fold lines
for apex in (368, 393):
    print(f"\n=== apex {apex}: pos={tuple(round(c,4) for c in verts[apex])} ===")
    fan = []
    for ti, t in enumerate(tris):
        if apex in t and fids[ti] == 3:
            fan.append((ti, t))
    print(f"plane-face-3 triangles at apex: {len(fan)}")
    # rim vertices = fan partners other than apex
    partners = defaultdict(int)
    for ti, t in fan:
        for v in t:
            if v != apex:
                partners[v] += 1
    rim = sorted(partners)
    print(f"partner verts: {len(rim)}: {rim[:40]}{'...' if len(rim)>40 else ''}")
    # heights of fan triangles vs their base edge
    hs = []
    for ti, t in fan:
        others = [v for v in t if v != apex]
        if len(others) != 2:
            continue
        b, c = others
        base = dist(b, c)
        ar = tri_area(apex, b, c)
        h = 2*ar/max(base, 1e-30)
        hs.append((h, base, t))
    hs.sort(reverse=True)
    print(f"fan tri heights: max={hs[0][0]:.4f} min={hs[-1][0]:.4f} n={len(hs)}")
    print(f"tallest 3: " + "; ".join(f"h={h:.4f} base={b:.4f} t={t}" for h, b, t in hs[:3]))
    # fan degree = number of triangles
    print(f"fan degree: {len(fan)}")

# --- plane face 3 domain stats
face3 = [t for ti, t in enumerate(tris) if fids[ti] == 3]
vv3 = set()
for t in face3:
    vv3.update(t)
print(f"\n=== plane face 3: tris={len(face3)} verts={len(vv3)} ===")
xs = [verts[v][0] for v in vv3]; ys = [verts[v][1] for v in vv3]; zs = [verts[v][2] for v in vv3]
print(f"bbox: x=[{min(xs):.4f},{max(xs):.4f}] y=[{min(ys):.4f},{max(ys):.4f}] z=[{min(zs):.4f},{max(zs):.4f}]")
extent = [max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs)]
thin = sorted(extent)[0]/max(sorted(extent)[1], 1e-30)
print(f"extent={tuple(round(e,4) for e in extent)} thinness={thin:.4f}")
# degree distribution of face-3 triangles per vertex
deg = defaultdict(int)
for t in face3:
    for v in t:
        deg[v] += 1
top = sorted(deg.items(), key=lambda kv: -kv[1])[:8]
print(f"top degrees: {[(v, d) for v, d in top]}")

# --- nurbs face 121
face121 = [t for ti, t in enumerate(tris) if fids[ti] == 121]
vv121 = set()
for t in face121:
    vv121.update(t)
print(f"\n=== nurbs face 121: tris={len(face121)} verts={len(vv121)} ===")
xs = [verts[v][0] for v in vv121]; ys = [verts[v][1] for v in vv121]; zs = [verts[v][2] for v in vv121]
print(f"bbox: x=[{min(xs):.4f},{max(xs):.4f}] y=[{min(ys):.4f},{max(ys):.4f}] z=[{min(zs):.4f},{max(zs):.4f}]")
# sliver analysis: heights of face-121 triangles
slivers = []
for ti, t in enumerate(face121):
    a, b, c = t
    e = [dist(a,b), dist(b,c), dist(c,a)]
    base = max(e)
    ar = tri_area(a, b, c)
    h = 2*ar/max(base, 1e-30)
    slivers.append((h, ar, t))
slivers.sort()
print(f"face-121 tri heights: min={slivers[0][0]:.6f} p10={slivers[len(slivers)//10][0]:.6f} median={slivers[len(slivers)//2][0]:.6f}")
print(f"thinnest 5: " + "; ".join(f"h={h:.6f} area={ar:.8f}" for h, ar, t in slivers[:5]))
n_thin = sum(1 for h, ar, t in slivers if h < 0.01)
print(f"tris with h<0.01: {n_thin}/{len(slivers)}")

# --- shared rim: plane fan base edges vs nurbs edges
rim_edges_plane = set()
for ti, t in enumerate(tris):
    if fids[ti] == 3 and 368 in t:
        others = [v for v in t if v != 368]
        if len(others) == 2:
            rim_edges_plane.add(tuple(sorted(others)))
nurbs_edges = defaultdict(int)
for ti, t in enumerate(face121):
    for a, b in ((t[0],t[1]),(t[1],t[2]),(t[2],t[0])):
        nurbs_edges[tuple(sorted((a,b)))] += 1
shared = rim_edges_plane & set(nurbs_edges)
print(f"\nfan-368 base edges: {len(rim_edges_plane)}, shared with nurbs-121: {len(shared)}")
