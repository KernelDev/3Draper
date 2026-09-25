#!/usr/bin/env python3
"""Exact per-face bnd/NM counts for selected faces across dumps."""
import sys, os, collections

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

faces = [198, 199, 200, 201, 202, 203, 204, 205, 206, 226, 102, 42, 38, 44]
for base in sys.argv[1:]:
    verts, tris, fid = load(base)
    usage = collections.Counter()
    for (a, b, c) in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            if u > v: u, v = v, u
            usage[(u, v)] += 1
    face_bnd = collections.Counter()
    face_nm = collections.Counter()
    for ti, (a, b, c) in enumerate(tris):
        f = fid.get(ti, -1)
        for u, v in ((a, b), (b, c), (c, a)):
            if u > v: u, v = v, u
            n = usage[(u, v)]
            if n == 1:
                face_bnd[f] += 1
            elif n >= 3:
                face_nm[f] += 1
    out = {f: (face_bnd.get(f, 0), face_nm.get(f, 0) // 3) for f in faces}
    print(os.path.basename(base), {f: f"bnd={b},nm={m}" for f, (b, m) in out.items()})
