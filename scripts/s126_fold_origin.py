#!/usr/bin/env python3
"""s126 final check: are the 15 genuine fold pairs (trigger-2 candidates
at after-gapfill) present in the RAW per-face emissions (pre-merge),
or created by the merge/weld chain (the s125 #10220 class)?

For each fold pair (from the pre-FIW mesh), locate both triangles in
the face emissions by position, and measure the dihedral there.
"""
import os, math
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDIR = f"{BASE}/forensics/s126/stages2"
EMDIR = f"{BASE}/forensics/s126/fm10212"

def load_obj(path):
    vs, fs = [], []
    fmap = {}
    for l in open(path):
        if l.startswith("v "):
            _, x, y, z = l.split()
            vs.append((float(x), float(y), float(z)))
        elif l.startswith("f "):
            _, a, b, c = l.split()
            fs.append((int(a)-1, int(b)-1, int(c)-1))
    for l in open(path.replace(".obj", ".fmap")):
        _, t, fid = l.split()
        fmap[int(t)] = int(fid)
    return vs, fs, fmap

vs, fs, fmap = load_obj(f"{SDIR}/p21_b10212-d-after-gapfill.obj")
e2 = defaultdict(list)
for ti, (a, b, c) in enumerate(fs):
    for v0, v1 in ((a, b), (b, c), (c, a)):
        e2[(min(v0, v1), max(v0, v1))].append(ti)
def edir(tri, x, y):
    a, b, c = tri
    for p, q in ((a, b), (b, c), (c, a)):
        if (p == x and q == y) or (p == y and q == x):
            return p
    return None
def normal(vs, tri):
    a, b, c = (vs[i] for i in tri)
    ux, uy, uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    vx, vy, vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
    return (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)
def tarea(vs, t):
    n = normal(vs, t)
    return 0.5*math.sqrt(n[0]**2+n[1]**2+n[2]**2)

# collect fold pairs (consistent-dir + >170 normals) pre-FIW
fold_pairs = []
for (ea, eb), tris in sorted(e2.items()):
    if len(tris) != 2:
        continue
    d0 = edir(fs[tris[0]], ea, eb)
    d1 = edir(fs[tris[1]], ea, eb)
    if d0 is None or d1 is None or d0 == d1:
        continue
    n0 = normal(vs, fs[tris[0]])
    n1 = normal(vs, fs[tris[1]])
    d = n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2]
    l0 = math.sqrt(n0[0]**2+n0[1]**2+n0[2]**2)
    l1 = math.sqrt(n1[0]**2+n1[1]**2+n1[2]**2)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, d/(l0*l1))))) if l0>1e-15 and l1>1e-15 else 0.0
    if ang > 170.0:
        elen = math.dist(vs[ea], vs[eb])
        h0 = 2*tarea(vs, fs[tris[0]])/elen if elen > 1e-12 else 9e9
        h1 = 2*tarea(vs, fs[tris[1]])/elen if elen > 1e-12 else 9e9
        fold_pairs.append((tris[0], tris[1], fmap.get(tris[0]), fmap.get(tris[1]),
                            ang, h0, h1))
print(f"fold pairs pre-FIW: {len(fold_pairs)}")

# load all face emissions (first-pass attempts only: a0..a19)
emissions = {}
for fn in sorted(os.listdir(EMDIR)):
    if not fn.startswith("mesh_brep10212_f"):
        continue
    parts = fn.replace(".txt", "").split("_")
    fid = int(parts[2][1:])
    att = int(parts[4].replace("a", ""))
    if att > 19:   # second pass — skip (p21 shipped)
        continue
    ev, et = [], []
    for l in open(f"{EMDIR}/{fn}"):
        if l.startswith("v "):
            _, _, x, y, z = l.split()
            ev.append((float(x), float(y), float(z)))
        elif l.startswith("t "):
            _, _, a, b, c = l.split()
            et.append((int(a), int(b), int(c)))
    emissions[fid] = (ev, et)

def find_tri(face, pts, tol=2e-3):
    """find emission triangle whose verts match the 3 positions"""
    ev, et = emissions.get(face, ([], []))
    def near(q):
        for i, p in enumerate(ev):
            if (abs(p[0]-q[0])<tol and abs(p[1]-q[1])<tol
                    and abs(p[2]-q[2])<tol):
                return i
        return None
    idx = [near(vs[p]) for p in pts]
    if any(i is None for i in idx):
        return None, idx
    s = set(idx)
    for t in et:
        if set(t) == s:
            return t, idx
    return None, idx

for t0, t1, f0, f1, ang, h0, h1 in fold_pairs:
    p0 = fs[t0]
    p1 = fs[t1]
    em0, idx0 = find_tri(f0, p0)
    em1, idx1 = find_tri(f1, p1)
    src0 = "EMISSION" if em0 else ("verts-only" if all(i is not None for i in idx0) else "ABSENT")
    src1 = "EMISSION" if em1 else ("verts-only" if all(i is not None for i in idx1) else "ABSENT")
    # dihedral in the raw emission if both found
    emang = None
    if em0 and em1:
        n0 = normal(emissions[f0][0], em0)
        n1 = normal(emissions[f1][0], em1)
        d = n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2]
        l0 = math.sqrt(n0[0]**2+n0[1]**2+n0[2]**2)
        l1 = math.sqrt(n1[0]**2+n1[1]**2+n1[2]**2)
        if l0 > 1e-15 and l1 > 1e-15:
            emang = math.degrees(math.acos(max(-1.0, min(1.0, d/(l0*l1)))))
    print(f"f({f0},{f1}) t{t0}/t{t1} merged: ang={ang:.1f} h=({h0:.3f},{h1:.3f})")
    print(f"   emission check: t0 {src0} (idx {idx0}), t1 {src1} (idx {idx1})"
          + (f", raw dihedral={emang:.1f}" if emang is not None else ""))
