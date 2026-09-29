#!/usr/bin/env python3
"""s65 diag 2: anatomy of the triple-face nm edges in #1086.

For the [1, cone, plane] groups: dump sample edges with all 3 triangles
(coords of the 3rd vertex, normal, which side of the edge it lies on).
Question: which face's triangle is the DOUBLE-COVERAGE one — does f1
(annulus) emit a strip along the cone-plane junction, or do cone/plane
overlap into each other?

Also: locate the junction geometrically (radius from axis, y-level) and
check the single nm edges [cone,plane] len=2.8868 + bnd edges of
f10/f11/f14.
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

def main():
    vs, tris, fids = load(OBJ)
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (u, v) if u < v else (v, u)
            e2t[e].append(ti)

    # face -> triangles
    ftris = defaultdict(list)
    for ti, f in fids.items():
        ftris[f].append(ti)

    print("=== face sizes (tris) around the junction ===")
    for f in [1, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21]:
        print(f"  f{f}: {len(ftris.get(f, []))} tris")

    # sample group [1,9,16]
    print("\n=== sample nm edges, group [1,9,16] ===")
    group = []
    for e, tl in e2t.items():
        fl = sorted({fids.get(t, -1) for t in tl})
        if fl == [1, 9, 16] and len(tl) == 3:
            group.append(e)
    group.sort()
    print(f"count={len(group)}")
    # vertex index range of the chain
    allv = sorted({v for e in group for v in e})
    print(f"vertex idx range: {allv[0]}..{allv[-1]}, unique={len(allv)}")

    def r_y_z(p):
        r = math.hypot(p[0], p[1])
        return f"r={r:.4f} y={p[2]:.4f} z={p[2]:.4f} ({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})"

    # walk the chain in order (each vertex has ≤2 group neighbors)
    adj = defaultdict(list)
    for e in group:
        adj[e[0]].append(e[1])
        adj[e[1]].append(e[0])
    ends = [v for v, n in adj.items() if len(n) == 1]
    chain = [ends[0]] if ends else [allv[0]]
    seen = {chain[0]}
    while True:
        nxt = [n for n in adj[chain[-1]] if n not in seen]
        if not nxt:
            break
        chain.append(nxt[0])
        seen.add(nxt[0])
    print(f"chain len={len(chain)} (edges {len(chain)-1})")
    print("chain endpoints:")
    for v in (chain[0], chain[-1]):
        print(f"  v{v}: {r_y_z(vs[v])}")
    print("chain midpoints (every 5th):")
    for i in range(0, len(chain), 5):
        v = chain[i]
        print(f"  v{v}: {r_y_z(vs[v])}")

    # for 3 sample edges (start/mid/end), dump all triangles
    samples = [group[0], group[len(group)//2], group[-1]]
    for e in samples:
        p0, p1 = vs[e[0]], vs[e[1]]
        print(f"\n--- edge {e}: len={math.dist(p0,p1):.4f} ---")
        print(f"  A: {r_y_z(p0)}")
        print(f"  B: {r_y_z(p1)}")
        # edge direction + normal in xy-plane for side test
        ex, ey = p1[0]-p0[0], p1[1]-p0[1]
        el = math.hypot(ex, ey)
        for t in e2t[e]:
            f = fids.get(t, -1)
            tri = tris[t]
            w = [v for v in tri if v not in e]
            if not w:
                print(f"    tri {t} face {f}: DEGENERATE (all verts on edge)")
                continue
            w = w[0]
            p = vs[w]
            # side: cross of edge dir with (w - p0)
            side = (ex*(p[1]-p0[1]) - ey*(p[0]-p0[0])) / (el if el else 1)
            # normal
            q0, q1, q2 = vs[tri[0]], vs[tri[1]], vs[tri[2]]
            ux, uy, uz = q1[0]-q0[0], q1[1]-q0[1], q1[2]-q0[2]
            vx, vy, vz = q2[0]-q0[0], q2[1]-q0[1], q2[2]-q0[2]
            nx, ny, nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
            nl = math.sqrt(nx*nx+ny*ny+nz*nz) or 1
            print(f"    tri {t} face {f}: apex v{w} {r_y_z(p)} side={side:+.4f} "
                  f"n=({nx/nl:+.3f},{ny/nl:+.3f},{nz/nl:+.3f})")

    # singles [9,16] etc
    print("\n=== single nm edges [cone,plane] len=2.8868 ===")
    for e, tl in e2t.items():
        fl = sorted({fids.get(t, -1) for t in tl})
        if len(tl) == 3 and len(fl) == 2:
            p0, p1 = vs[e[0]], vs[e[1]]
            print(f"edge {e} faces={fl} len={math.dist(p0,p1):.4f}")
            print(f"  A: {r_y_z(p0)}")
            print(f"  B: {r_y_z(p1)}")
            for t in tl:
                f = fids.get(t, -1)
                tri = tris[t]
                w = [v for v in tri if v not in e]
                if w:
                    print(f"    tri {t} face {f}: apex {r_y_z(vs[w[0]])}")

    # bnd edges f10/f11/f14
    print("\n=== bnd edges f10/f11/f14 ===")
    for e, tl in e2t.items():
        if len(tl) == 1:
            f = fids.get(tl[0], -1)
            if f in (10, 11, 14):
                p0, p1 = vs[e[0]], vs[e[1]]
                print(f"edge {e} face {f} len={math.dist(p0,p1):.4f}")
                print(f"  A: {r_y_z(p0)}")
                print(f"  B: {r_y_z(p1)}")
                tri = tris[tl[0]]
                w = [v for v in tri if v not in e]
                if w:
                    print(f"    owner tri apex: {r_y_z(vs[w[0]])}")

if __name__ == "__main__":
    main()
