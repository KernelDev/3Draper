#!/usr/bin/env python3
"""s127: chordal deviation (sagitta) of f2 emission vs the true cylinder.
For each triangle, sample centroid + edge midpoints; deviation = |radial - 2.5|.
Compare legacy (s126 dump) vs CDT (fm_after2)."""
import math, sys

O = (0.0, -1.675, 0.0)
R = 2.5

def load(path):
    verts, tris = [], []
    for l in open(path):
        p = l.split()
        if not p:
            continue
        if p[0] == "v":
            verts.append(tuple(float(x) for x in p[2:5]))
        elif p[0] == "t":
            tris.append((int(p[2]), int(p[3]), int(p[4])))
    return verts, tris

def dev(p):
    return abs(math.hypot(p[0]-O[0], p[2]-O[2]) - R)

def sag(path, tag):
    verts, tris = load(path)
    mx = 0.0
    argmax = None
    for ti, t in enumerate(tris):
        a, b, c = (verts[i] for i in t)
        samples = [
            ((a[0]+b[0]+c[0])/3, (a[1]+b[1]+c[1])/3, (a[2]+b[2]+c[2])/3),
            ((a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2),
            ((b[0]+c[0])/2, (b[1]+c[1])/2, (b[2]+c[2])/2),
            ((c[0]+a[0])/2, (c[1]+a[1])/2, (c[2]+a[2])/2),
        ]
        for s in samples:
            d = dev(s)
            if d > mx:
                mx = d
                argmax = (ti, s)
    print(f"{tag}: tris={len(tris)} verts={len(verts)} max_sagitta={mx:.6f} at tri {argmax[0]}")
    return mx

if __name__ == "__main__":
    sag("forensics/s126/fm10212/mesh_brep10212_f2_Cylinder_a1.txt", "f2 LEGACY (s126)")
    sag("forensics/s127/fm_after2/mesh_brep10212_f2_Cylinder_a1.txt", "f2 CDT (s127)")
    sag("forensics/s126/fm10212/mesh_brep10212_f19_Cylinder_a18.txt", "f19 LEGACY (s126)")
    sag("forensics/s127/fm_after2/mesh_brep10212_f19_Cylinder_a18.txt", "f19 CDT (s127)")
