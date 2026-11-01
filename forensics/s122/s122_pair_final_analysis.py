#!/usr/bin/env python3
"""s122-п.3 final-mesh analysis of the #31359 Cone(7)×Nurbs(11) pair.

Works in the INSTANCE frame (brep2 = NAUO_15973). Finds the pair's
triangles (fid 7 and 11 sharing an edge), prints exact geometry
(9-decimal), then answers:
  1. exact h over the shared edge for both apexes;
  2. the full triangle neighborhoods of both apex vertices (are they
     used by other triangles of either face? — sliver-vs-fan context);
  3. which junction-curve points the shared edge consists of and the
     cone-side span (how many junction segments the needle covers);
  4. whether the nurbs apex is a weld survivor (check the local-frame
     FACE_OBJS emission for a matching point under the derived rigid
     transform — approximate check via distance ratios).
"""
import math, itertools

def load(path):
    verts, tris = [], []
    for line in open(path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
            tris.append((a, b, c))
    return verts, tris

def load_fmap(path):
    fid = {}
    for line in open(path):
        p = line.split()
        if len(p) >= 3 and p[0] == "t":
            fid[int(p[1])] = int(p[2])
    return fid

def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def norm(a): return math.sqrt(sum(x*x for x in a))

base = "/tmp/my-project/forensics_s122/finalobjs/brep2_NAUO_15973 (BREP#31359)"
verts, tris = load(base + ".obj")
fids = load_fmap(base + ".fmap")

# The census pair: T360 (fid 7) and T1171 (fid 11) — find them by shared
# vertices at (2.6074,0.3424,0.9445)/(2.6201,0.3281,0.9445).
A = (2.6074, 0.3424, 0.9445)
B = (2.6201, 0.3281, 0.9445)
def key4(v): return tuple(f"{c:.4f}" for c in v)
va = [i for i, v in enumerate(verts) if key4(v) == key4(A)]
vb = [i for i, v in enumerate(verts) if key4(v) == key4(B)]
print(f"shared-edge vertex ids: A={va} B={vb}")
for i in va + vb:
    print(f"  v{i} = ({verts[i][0]:.9f}, {verts[i][1]:.9f}, {verts[i][2]:.9f})")

pair = []
for ti, t in enumerate(tris):
    s = set(t)
    if va and vb and (s & set(va)) and (s & set(vb)):
        pair.append((ti, t, fids.get(ti)))
print(f"\ntriangles on the shared edge: {len(pair)}")
for ti, t, fid in pair:
    third = [x for x in t if x not in va + vb]
    tv = verts[third[0]] if third else None
    h = norm(cross(sub(tv, verts[va[0]]), sub(vb[0] and verts[vb[0]] or verts[vb[-1]], verts[va[0]]))) / norm(sub(verts[vb[0]], verts[va[0]])) if tv else float("nan")
    print(f"  tri#{ti} fid={fid} verts={t} third=({tv[0]:.9f},{tv[1]:.9f},{tv[2]:.9f}) h_over_edge={h:.6f}")

# neighborhoods of the apexes
for ti, t, fid in pair:
    third = [x for x in t if x not in va + vb][0]
    users = [j for j, tt in enumerate(tris) if third in tt]
    fsum = {}
    for j in users:
        f = fids.get(j)
        fsum[f] = fsum.get(f, 0) + 1
    print(f"\napex v{third} used by {len(users)} tris, by fid: {fsum}")

# the junction curve in the instance frame: walk vertices near the plane
# z=0.9445 with x in [2.4, 2.9] sorted by angle around the cone axis
cand = [i for i, v in enumerate(verts) if abs(v[2] - 0.9445) < 5e-4 and 2.3 < v[0] < 2.9]
cand.sort(key=lambda i: math.atan2(verts[i][1], verts[i][0]))
print(f"\ninstance junction-plane points (z≈0.9445): {len(cand)}")
for i in cand:
    users = [j for j, tt in enumerate(tris) if i in tt]
    fsum = {}
    for j in users:
        f = fids.get(j)
        fsum[f] = fsum.get(f, 0) + 1
    print(f"  v{i} ({verts[i][0]:.6f},{verts[i][1]:.6f},{verts[i][2]:.6f}) used_by={fsum}")
