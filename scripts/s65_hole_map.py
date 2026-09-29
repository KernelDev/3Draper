#!/usr/bin/env python3
"""s65 diag 8: map f9's triangles around the hole loop — which triangles
touch the loop vertices, what regions are covered, and where is the
empty area. Also dump the UV logic: the ring at constant cone-v
(r=2.7103, y=4.8939) vs the lip curve — where does the ring poke
below the lip?
"""
import math
import glob
from collections import defaultdict

OBJ = glob.glob("/tmp/s65_objs/brep23_*.obj")[0]

def load(obj_path):
    vs, tris = [], []
    for line in open(obj_path):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(p.split("/")[0]) - 1 for p in line.split()[1:4])
            tris.append((a, b, c))
    fids = {}
    for line in open(obj_path.replace(".obj", ".fmap")):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fids[int(ti)] = int(fid)
    return vs, tris, fids

def local(p):
    return (-660.6873 - p[1], p[0] + 45.0, p[2] - 520.5)

def main():
    vs, tris, fids = load(OBJ)
    f9 = [ti for ti, f in fids.items() if f == 9]
    print(f"f9 tris: {len(f9)}")

    # lip curve y at given azimuth (Bézier #5577, part frame):
    # cps: (2.8868,5,0), (2.4162,4.7173,-0.815), (1.9146,4.7168,-1.6838), (1.4434,5,-2.5)
    cps = [(2.8868, 5.0, 0.0), (2.4162, 4.7173, -0.815),
           (1.9146, 4.7168, -1.6838), (1.4434, 5.0, -2.5)]
    def bez(t):
        out = [0.0, 0.0, 0.0]
        for i, c in enumerate(cps):
            b = math.comb(3, i) * (1-t)**(3-i) * t**i
            for k in range(3):
                out[k] += b * c[k]
        return out
    print("lip curve y vs ring y=4.8939 by azimuth:")
    for azd in range(0, -61, -5):
        # find t where azimuth matches
        best = None
        for i in range(101):
            t = i / 100
            x, y, z = bez(t)
            a = math.degrees(math.atan2(z, x))
            if best is None or abs(a - azd) < abs(best[1] - azd):
                best = (t, a, y)
        ring = 4.8939
        print(f"  az={azd:4d}: lip y={best[2]:.4f} ring y={ring:.4f} "
              f"{'RING BELOW lip (outside lune)' if ring < best[2] else 'ring above lip (inside)'}")

    # triangles touching the hole loop verts
    loop = [187, 1061, 1067, 1066, 1065, 1064, 1063, 1062, 1005, 1006, 1007]
    print("\ntriangles of f9 touching loop verts:")
    for ti in sorted(f9):
        t = tris[ti]
        hits = [v for v in loop if v in t]
        if hits:
            pts = [local(vs[v]) for v in t]
            desc = " ".join(f"({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})" for p in pts)
            print(f"  tri {ti}: verts {t} -> {desc}")

if __name__ == "__main__":
    main()
