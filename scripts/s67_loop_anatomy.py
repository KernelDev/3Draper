#!/usr/bin/env python3
"""s67 diag 2: loop anatomy of top-bnd Torus faces in drill HOUSING.

For each selected face: chain its bnd edges into connected components,
report each loop: #edges, #verts, closed/open, total 3D length, bbox,
edge-length stats (min/med/max), and vertical (axis) extent.
Also dumps first 3 edge lengths to spot discretization mismatch.
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP, FMAPF = BASE + ".obj", BASE + ".fmap", BASE + ".facemap"

# top Torus faces from census + worst cylinders
TARGETS = [236, 238, 212, 127, 166, 160, 162, 158, 164, 130, 125, 214]


def main():
    verts, tris = [], []
    for line in open(OBJ):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(t) - 1 for t in line.split()[1:4])
            tris.append((a, b, c))
    fid_of = {}
    for line in open(FMAP):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of[int(ti)] = int(fid)
    stype = {}
    for line in open(FMAPF):
        if line.startswith("f "):
            parts = line.split()
            fid = int(parts[1])
            stype[fid] = " ".join(parts[2:-2])

    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
    face_bnd = defaultdict(list)
    for e, ts in edge_tris.items():
        if len(ts) == 1:
            face_bnd[fid_of.get(ts[0], -1)].append(e)

    for fid in TARGETS:
        edges = face_bnd.get(fid, [])
        st = stype.get(fid, "?")
        print(f"\n===== fid={fid} [{st}] : {len(edges)} bnd =====")
        adj = defaultdict(list)
        for (u, v) in edges:
            adj[u].append(v)
            adj[v].append(u)
        seen = set()
        loops = []
        for s in adj:
            if s in seen:
                continue
            stack, comp = [s], []
            seen.add(s)
            while stack:
                n = stack.pop()
                comp.append(n)
                for m in adj[n]:
                    if m not in seen:
                        seen.add(m)
                        stack.append(m)
            loops.append(comp)
        loops.sort(key=len, reverse=True)
        for li, comp in enumerate(loops):
            ces = [e for e in edges if e[0] in comp]
            lens = [math.dist(verts[u], verts[v]) for (u, v) in ces]
            pts = [verts[v] for v in comp]
            xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]
            closed = any(m in comp for (u, v) in edges
                         for m in [] ) # placeholder
            # closed if any vertex has degree != 2 → open chain ends
            degs = defaultdict(int)
            for (u, v) in ces:
                degs[u] += 1; degs[v] += 1
            ends = [v for v in comp if degs[v] == 1]
            lens.sort()
            print(f"  loop{li}: {len(comp):4d} verts, {len(ces):4d} edges, "
                  f"{'OPEN' if ends else 'closed'}, len={sum(lens):8.3f}, "
                  f"edge[min/med/max]={lens[0]:.4f}/{lens[len(lens)//2]:.4f}/"
                  f"{lens[-1]:.4f}, bbox "
                  f"x[{min(xs):7.2f},{max(xs):7.2f}] "
                  f"y[{min(ys):7.2f},{max(ys):7.2f}] "
                  f"z[{min(zs):7.2f},{max(zs):7.2f}]")


if __name__ == "__main__":
    main()
