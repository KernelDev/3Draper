#!/usr/bin/env python3
"""Per-face min/median triangle 3D area on a final OBJ dump."""
import sys, os, collections, math

def load(base):
    obj = fmap = None
    for f in os.listdir(base):
        if f.endswith('.obj'):
            obj = os.path.join(base, f)
        if f.endswith('.fmap'):
            fmap = os.path.join(base, f)
    verts, tris = [], []
    with open(obj) as fh:
        for line in fh:
            if line.startswith('v '):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith('f '):
                p = line.split()[1:]
                tris.append((int(p[0])-1, int(p[1])-1, int(p[2])-1))
    fid = {}
    with open(fmap) as fh:
        for line in fh:
            if line.startswith('t '):
                _, ti, f = line.split()
                fid[int(ti)] = int(f)
    return verts, tris, fid

def area(v, t):
    a, b, c = v[t[0]], v[t[1]], v[t[2]]
    ab = tuple(b[i]-a[i] for i in range(3))
    ac = tuple(c[i]-a[i] for i in range(3))
    cx = ab[1]*ac[2]-ab[2]*ac[1]; cy = ab[2]*ac[0]-ab[0]*ac[2]; cz = ab[0]*ac[1]-ab[1]*ac[0]
    return 0.5*math.sqrt(cx*cx+cy*cy+cz*cz)

faces = [int(a) for a in sys.argv[2:]] if len(sys.argv) > 2 else [198,200,202,204,206,226,102,42,38,44]
verts, tris, fid = load(sys.argv[1])
by_face = collections.defaultdict(list)
for ti, t in enumerate(tris):
    f = fid.get(ti, -1)
    if f in faces:
        by_face[f].append(area(verts, t))
print(f"{'face':>6} {'n_tri':>6} {'min_area':>10} {'p10':>10} {'med':>10}")
for f in sorted(by_face):
    a = sorted(by_face[f])
    n = len(a)
    print(f"f{f:<5} {n:>6} {a[0]:>10.3e} {a[n//10]:>10.3e} {a[n//2]:>10.3e}")
