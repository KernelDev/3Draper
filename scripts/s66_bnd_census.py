#!/usr/bin/env python3
"""s66 diag 1: boundary-edge census of BREP#1083 ELBETAETIGUNG.

Parses the DRAPER_DUMP_FINAL_OBJS dump (brep19) — final post-repair
instance mesh — finds boundary edges (used by exactly 1 triangle),
groups them by source face id (fmap), chains into loops, prints
per-face stats + 3D geometry of each loop.
"""
import math
from collections import defaultdict

OBJ = ("/home/z/my-project/scripts/s66_objs/"
       "brep19_ELBETAETIGUNG_WWK-017863-KON-A (BREP#1083).obj")
FMAP = OBJ.replace(".obj", ".fmap")


def main():
    verts = []
    tris = []
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
    print(f"mesh: {len(verts)} verts, {len(tris)} tris, "
          f"{len(set(fid_of.values()))} face ids")

    edge_tris = defaultdict(list)
    edge_fids = defaultdict(set)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
            edge_fids[e].add(fid_of.get(ti, -1))

    bnd = [e for e, ts in edge_tris.items() if len(ts) == 1]
    nm = [e for e, ts in edge_tris.items() if len(ts) > 2]
    print(f"edges: {len(edge_tris)} total | boundary {len(bnd)} | "
          f"non-manifold {len(nm)}")

    # per-face census (boundary edge belongs to its single triangle's face)
    face_bnd = defaultdict(list)
    for e in bnd:
        face_bnd[list(edge_fids[e])[0]].append(e)
    print("\nper-face boundary census (fid: count, chained loops):")
    for fid in sorted(face_bnd, key=lambda f: -len(face_bnd[f])):
        edges = face_bnd[fid]
        # chain into loops
        adj = defaultdict(list)
        for (u, v) in edges:
            adj[u].append(v)
            adj[v].append(u)
        loops = []
        seen_e = set()
        for start in sorted(adj):
            if adj[start] and all(len(adj[n]) == 2 for n in adj):
                pass  # chain walk below
        visited = set()
        for start in sorted(adj):
            if start in visited:
                continue
            # walk the chain
            loop = [start]
            visited.add(start)
            prev, cur = None, start
            while True:
                nxts = [n for n in adj[cur] if n != prev]
                nxt = nxts[0] if nxts else None
                if nxt is None or nxt == start or nxt in visited:
                    break
                loop.append(nxt)
                visited.add(nxt)
                prev, cur = cur, nxt
            loops.append(loop)
        lens = []
        for lp in loops:
            L = 0.0
            for i in range(len(lp)):
                u, v = lp[i], lp[(i + 1) % len(lp)]
                L += math.dist(verts[u], verts[v])
            lens.append(L)
        bboxes = []
        for lp in loops:
            xs = [verts[i][0] for i in lp]; ys = [verts[i][1] for i in lp]
            zs = [verts[i][2] for i in lp]
            bboxes.append(f"bbox[{min(xs):.2f},{min(ys):.2f},{min(zs):.2f}"
                          f"-> {max(xs):.2f},{max(ys):.2f},{max(zs):.2f}]")
        print(f"  fid={fid}: {len(edges)} bnd edges, {len(loops)} loops")
        for lp, L, bb in zip(loops, lens, bboxes):
            print(f"    loop {len(lp)} pts, len={L:.4f}, {bb}")
            pts = [verts[i] for i in lp]
            # print first 3 + last 3 points compactly
            for p in pts[:3]:
                print(f"      ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
            if len(pts) > 6:
                print("      ...")
            for p in pts[-3:]:
                print(f"      ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
        # edge length stats
        elens = sorted(math.dist(verts[u], verts[v]) for (u, v) in edges)
        if elens:
            print(f"    edge len: min={elens[0]:.5f} med={elens[len(elens)//2]:.5f} "
                  f"max={elens[-1]:.5f}")


if __name__ == "__main__":
    main()
