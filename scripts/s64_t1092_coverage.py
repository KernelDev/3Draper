#!/usr/bin/env python3
"""s64 diag: coverage map of f29/f32 per-face meshes in (angle, y) space.

The cylinder f29: axis@ (17,·,42) dir +y, r=3. u=angle [0..360) around
axis (ref=(0,0,1)); v=y. Dumps a coarse ASCII map of which (u,v) cells
the mesh triangles cover, plus the boundary loop structure.
"""
import math
from collections import defaultdict

D = "/tmp/s64_faces"

def load(path):
    verts, tris = [], []
    for line in open(path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            tris.append(tuple(int(p) - 1 for p in line.split()[1:4]))
    return verts, tris

def main():
    for fname, ffile, cx, cz in [
        ("f29", "brep1092_f29_s1831_Cylinder.obj", 17.0, 42.0),
        ("f32", "brep1092_f32_s1834_Cylinder.obj", -17.0, 42.0),
    ]:
        verts, tris = load(f"{D}/{ffile}")
        print(f"\n===== {fname}: v={len(verts)} t={len(tris)}")
        # u = angle of (x-cx, z-cz); v = y
        us = []
        for (x, y, z) in verts:
            a = math.degrees(math.atan2(z - cz, x - cx)) % 360
            us.append((a, y))
        # triangle centroid coverage grid
        NU, NV = 36, 14
        grid = [[0] * NU for _ in range(NV)]
        ymin, ymax = 24.0, 41.0
        for (a, b, c) in tris:
            pts = []
            for i in (a, b, c):
                x, y, z = verts[i]
                ang = math.degrees(math.atan2(z - cz, x - cx)) % 360
                pts.append((ang, y))
            # handle angle wraparound by triangle: if span > 180, shift
            angs = [p[0] for p in pts]
            if max(angs) - min(angs) > 180:
                pts = [((p[0] + 180) % 360 - 180, p[1]) for p in pts]
            for k in range(3):
                p0 = pts[k]; p1 = pts[(k + 1) % 3]; p2 = pts[(k + 2) % 3]
                for t1 in range(0, 4):
                    for t2 in range(0, 4 - t1):
                        w1 = t1 / 3.0; w2 = t2 / 3.0; w0 = 1 - w1 - w2
                        au = w0 * p0[0] + w1 * p1[0] + w2 * p2[0]
                        av = w0 * p0[1] + w1 * p1[1] + w2 * p2[1]
                        if av < ymin or av > ymax:
                            continue
                        au %= 360
                        iu = int(au / 360 * NU); iv = int((av - ymin) / (ymax - ymin) * NV)
                        if 0 <= iu < NU and 0 <= iv < NV:
                            grid[iv][iu] += 1
        print("y\\u  " + "".join(f"{int(i * 10):<2d}"[:2] for i in range(NU)) + "   (u tens of deg)")
        for iv in range(NV - 1, -1, -1):
            y = ymin + (iv + 0.5) / NV * (ymax - ymin)
            row = "".join("#" if grid[iv][iu] else "." for iu in range(NU))
            print(f"{y:5.1f} {row}")
        # boundary loop in (u,y)
        cnt = defaultdict(int)
        for (a, b, c) in tris:
            for u, v in ((a, b), (b, c), (c, a)):
                e = (u, v) if u < v else (v, u)
                cnt[e] += 1
        bed = [e for e, n in cnt.items() if n == 1]
        bv = sorted({v for e in bed for v in e})
        print(f"bnd_edges={len(bed)} bnd_verts={len(bv)}")
        # histogram of boundary verts by y band
        bands = defaultdict(int)
        for v in bv:
            bands[int(verts[v][1])] += 1
        print("bnd verts by y:", dict(sorted(bands.items())))

if __name__ == "__main__":
    main()
