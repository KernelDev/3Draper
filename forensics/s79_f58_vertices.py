#!/usr/bin/env python3
"""s79: exact 3D vertices of the (57,58) fold triangles + the f57
fan structure + the shared-edge location."""
import math
from collections import defaultdict

OBJ = ("/tmp/s79_final/brep4_HOUSING_MIRROR (BREP#62542).obj")
FMAP = ("/tmp/s79_final/brep4_HOUSING_MIRROR (BREP#62542).fmap")


def load():
    verts, tris = [], []
    with open(OBJ) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                a, b, c = [int(x) for x in line.split()[1:4]]
                tris.append((a - 1, b - 1, c - 1))
    t2f = {}
    with open(FMAP) as f:
        for line in f:
            if line.startswith("t "):
                _, ti, fid = line.split()
                t2f[int(ti)] = int(fid)
    return verts, tris, t2f


def area(verts, t):
    a, b, c = verts[t[0]], verts[t[1]], verts[t[2]]
    u = [b[k] - a[k] for k in range(3)]
    v = [c[k] - a[k] for k in range(3)]
    n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
    return math.sqrt(sum(x*x for x in n)) / 2


def main():
    verts, tris, t2f = load()

    # the six worst (57,58) pairs from the earlier analysis
    pairs = [
        ([7585, 7596, 7597], [7597, 7596, 7598]),
        ([7521, 7522, 7537], [7537, 7522, 7538]),
    ]
    # re-extract from probe file instead
    import re
    RE = re.compile(
        r"faces=\(57,58\).*tris=\((\[[^]]*\]),(\[[^]]*\])\).*"
        r"h=\(([^,]+),([^)]+)\)")
    with open("forensics/s79_drill_full.txt") as f:
        n = 0
        for line in f:
            if "SUBTOL" in line or "faces=(57,58)" not in line:
                continue
            m = RE.search(line)
            if not m:
                continue
            h = (float(m.group(3)), float(m.group(4)))
            if max(h) < 1.0:
                continue
            ta = [int(x) for x in m.group(1)[1:-1].split(",")]
            tb = [int(x) for x in m.group(2)[1:-1].split(",")]
            print(f"--- pair h=({h[0]:.3f},{h[1]:.3f}) ---")
            for nm, t in (("f57Plane", ta), ("f58Nurbs", tb)):
                vs = [verts[i] for i in t]
                print(f"  {nm} tri{t}: area={area(verts, t):.6f}")
                for v in vs:
                    print(f"      v=({v[0]:.4f},{v[1]:.4f},{v[2]:.4f})")
            shared = set(ta) & set(tb)
            print(f"  shared verts: {shared} at "
                  f"{[verts[i] for i in shared]}")
            n += 1
            if n >= 3:
                break

    # f57 plane: what does it look like — a fan? vertex degree census
    f2t = defaultdict(list)
    for ti, fid in t2f.items():
        f2t[fid].append(ti)
    f57 = f2t[57]
    deg = defaultdict(int)
    for ti in f57:
        for v in tris[ti]:
            deg[v] += 1
    top = sorted(deg.items(), key=lambda kv: -kv[1])[:5]
    print(f"\nf57: {len(f57)} tris, top-degrees: {top}")
    # where are the top-degree vertices?
    for v, d in top:
        print(f"   v{v} deg={d} at {verts[v]}")

    # f58 ladder: connectivity between rim verts and band verts
    f58 = f2t[58]
    # find band verts = verts used ONLY by f58 (not by f57/f26 etc)
    f57_v = set()
    for ti in f57:
        f57_v.update(tris[ti])
    f58_v = set()
    for ti in f58:
        f58_v.update(tris[ti])
    only58 = f58_v - f57_v
    print(f"\nf58: {len(f58)} tris, {len(f58_v)} verts, "
          f"{len(f58_v - f57_v)} not shared with f57")
    # triangle count by composition
    comp = defaultdict(int)
    for ti in f58:
        k = sum(1 for v in tris[ti] if v in f57_v)
        comp[k] += 1
    print(f"f58 tris by #shared-with-f57 verts: {dict(comp)}")


if __name__ == "__main__":
    main()
