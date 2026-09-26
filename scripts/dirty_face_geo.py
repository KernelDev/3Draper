#!/usr/bin/env python3
"""Session-57: localize pre-merge FO pairs on a dirty face (torus/cyl).

For every same-side overlap pair: shared-edge length, azimuthal span (in
lattice steps), radial/z extents, and the azimuth distribution (seam
clustering?). Assumes the face is a surface of revolution about the Z
axis through the vertex centroid's XY (auto-fit).

Usage: dirty_face_geo.py <obj> [--axis auto]
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

obj = Path(sys.argv[1])
verts, tris = [], []
for line in obj.read_text().splitlines():
    if line.startswith("v "):
        _, x, y, z = line.split()
        verts.append((float(x), float(y), float(z)))
    elif line.startswith("f "):
        a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
        tris.append((a, b, c))

# Axis = Z through centroid XY (surfaces of revolution in drill are
# axis-aligned; centroid XY is the axis for a full ring).
cx = sum(v[0] for v in verts) / len(verts)
cy = sum(v[1] for v in verts) / len(verts)


def cyl(v):
    dx, dy = v[0] - cx, v[1] - cy
    return math.atan2(dy, dx), math.hypot(dx, dy), v[2]


cyls = [cyl(v) for v in verts]

edge_tris = defaultdict(list)
for ti, (a, b, c) in enumerate(tris):
    for e in ((a, b), (b, c), (c, a)):
        edge_tris[(min(e), max(e))].append((ti, e[0], e[1]))


def sub(p, q):
    return (p[0] - q[0], p[1] - q[1], p[2] - q[2])


def cross(p, q):
    return (p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0])


def dot(p, q):
    return p[0] * q[0] + p[1] * q[1] + p[2] * q[2]


# Lattice steps: estimate from all edges' azimuthal/radial/z deltas.
az_steps = []
fo_pairs = []
for (e, tl) in edge_tris.items():
    if len(tl) != 2:
        continue
    (t0, a0, b0), (t1, a1, b1) = tl
    a, b = verts[e[0]], verts[e[1]]
    ed = sub(b, a)

    def apex_side(ti_, e0, e1):
        for v in tris[ti_]:
            if v != e0 and v != e1:
                return cross(ed, sub(verts[v], a)), v
        return (0.0, 0.0, 0.0), e0

    s0, ap0 = apex_side(t0, e[0], e[1])
    s1, ap1 = apex_side(t1, e[0], e[1])
    if dot(s0, s1) <= 0:
        continue
    # overlap pair: measure
    eLen = math.sqrt(dot(ed, ed))
    u0, r0, z0 = cyls[e[0]]
    u1, r1, z1 = cyls[e[1]]
    du = abs(u1 - u0)
    if du > math.pi:
        du = 2 * math.pi - du
    fo_pairs.append({
        "e": e, "len": eLen, "du": math.degrees(du),
        "dr": abs(r1 - r0), "dz": abs(z1 - z0),
        "u": math.degrees((u0 + u1) / 2 if abs(u0 - u1) < math.pi else (u0 + u1) / 2 - math.pi),
        "ap": (ap0, ap1),
    })

print(f"verts={len(verts)} tris={len(tris)} FO_pairs={len(fo_pairs)}")
if not fo_pairs:
    sys.exit(0)

# edge length stats vs lattice step
lens = sorted(p["len"] for p in fo_pairs)
dus = sorted(p["du"] for p in fo_pairs)
n = len(lens)
print(f"edge len: min={lens[0]:.4f} p25={lens[n//4]:.4f} med={lens[n//2]:.4f} p75={lens[3*n//4]:.4f} max={lens[-1]:.4f}")
print(f"az span (deg): min={dus[0]:.2f} med={dus[n//2]:.2f} max={dus[-1]:.2f}")

# typical minimal lattice step from ALL mesh edges (shortest quartile)
all_lens = []
for (e, tl) in edge_tris.items():
    a, b = verts[e[0]], verts[e[1]]
    all_lens.append(math.sqrt(dot(sub(b, a), sub(b, a))))
all_lens.sort()
m = len(all_lens)
print(f"mesh edges: med={all_lens[m//2]:.4f} p10={all_lens[m//10]:.4f} p90={all_lens[9*m//10]:.4f}")

# azimuth histogram of FO pairs (24 bins)
hist = [0] * 24
for p in fo_pairs:
    b = int(((p["u"] % 360) + 360) % 360 / 15)
    hist[b] += 1
print("azimuth hist (15° bins):", hist)

# how many FO edges are "long" (> 3× median mesh edge)
thr = 3 * all_lens[m // 2]
print(f"FO edges longer than 3*med_edge ({thr:.4f}): {sum(1 for p in fo_pairs if p['len'] > thr)}/{len(fo_pairs)}")

# radial-only vs z-span edges
print(f"FO edges with dz < 1e-9 (same ring): {sum(1 for p in fo_pairs if p['dz'] < 1e-9)}")
print(f"FO edges with dr < 1e-9: {sum(1 for p in fo_pairs if p['dr'] < 1e-9)}")

# apex pattern: for the top-5 longest, print both triangles' vertices in cylindrical coords
for p in sorted(fo_pairs, key=lambda q: -q["len"])[:5]:
    print(f"\nlong FO edge: len={p['len']:.4f} du={p['du']:.1f}° dr={p['dr']:.4f} dz={p['dz']:.4f}")
    for ti_ap in [(0, p['e'], p['ap'][0]), (1, p['e'], p['ap'][1])]:
        pass
    t0v = [v for v in tris[edge_tris[p['e']][0][0]]]
    t1v = [v for v in tris[edge_tris[p['e']][1][0]]]
    print("  T0:", " ".join(f"u={math.degrees(cyls[v][0])%360:7.2f} r={cyls[v][1]:.4f} z={cyls[v][2]:.4f}" for v in t0v))
    print("  T1:", " ".join(f"u={math.degrees(cyls[v][0])%360:7.2f} r={cyls[v][1]:.4f} z={cyls[v][2]:.4f}" for v in t1v))
