#!/usr/bin/env python3
"""s67 diag 1: bnd-edge census of drill HOUSING (BREP#47598), 27451 bnd.

Loads DRAPPER_DUMP_FINAL_OBJS dump (obj + fmap + facemap), finds
boundary edges (used by exactly 1 triangle), groups them by source
face + surface type, chains into loops, prints:
  1. totals (bnd / nm) vs the converter's NOT-watertight line
  2. bnd by surface-type class ( Nurbs vs Torus vs Plane vs Cylinder...)
  3. top-20 faces by bnd count with loop counts and lengths
"""
import math
from collections import defaultdict

BASE = ("/home/z/my-project/scripts/s67_objs/"
        "brep3_HOUSING (BREP#47598)")
OBJ, FMAP, FMAPF = BASE + ".obj", BASE + ".fmap", BASE + ".facemap"


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
    stype, sfid = {}, {}
    for line in open(FMAPF):
        if line.startswith("f "):
            parts = line.split()
            # f <fid> <stype...with spaces...> <step_id> <fwd>
            fid = int(parts[1])
            st = " ".join(parts[2:-2])
            sfid[fid] = int(parts[-2])
            stype[fid] = st
    print(f"mesh: {len(verts)} verts, {len(tris)} tris, "
          f"{len(stype)} faces mapped")

    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
    bnd = [e for e, ts in edge_tris.items() if len(ts) == 1]
    nm = [e for e, ts in edge_tris.items() if len(ts) > 2]
    print(f"edges: {len(edge_tris)} | boundary {len(bnd)} | "
          f"non-manifold {len(nm)}")

    # bnd edge -> face of its single triangle
    face_bnd = defaultdict(list)
    for e in bnd:
        face_bnd[fid_of.get(edge_tris[e][0], -1)].append(e)

    # 1) by surface class
    cls_bnd = defaultdict(int)
    cls_faces = defaultdict(set)
    for fid, edges in face_bnd.items():
        key = stype.get(fid, "?unknown")
        cls_bnd[key] += len(edges)
        cls_faces[key].add(fid)
    print("\n=== bnd by surface type ===")
    for k in sorted(cls_bnd, key=lambda k: -cls_bnd[k]):
        print(f"  {k:42s} : {cls_bnd[k]:6d} bnd on "
              f"{len(cls_faces[k]):3d} faces")

    # 2) top-20 faces
    print("\n=== top-20 faces by bnd ===")
    top = sorted(face_bnd, key=lambda f: -len(face_bnd[f]))[:20]
    for fid in top:
        edges = face_bnd[fid]
        adj = defaultdict(list)
        for (u, v) in edges:
            adj[u].append(v)
            adj[v].append(u)
        # chain loops
        seen, loops = set(), []
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
        lens = []
        for comp in loops:
            per = sum(
                math.dist(verts[u], verts[v])
                for (u, v) in edges if u in comp
            )
            lens.append(per)
        print(f"  fid={fid:5d} step={sfid.get(fid, 0):6d} "
              f"{stype.get(fid, '?'):30s} : {len(edges):5d} bnd, "
              f"{len(loops):3d} loops, total-len "
              f"{sum(lens):9.2f}")

    # 3) STEP face ids of top faces for cross-referencing
    print("\ntop-20 STEP face ids:",
          sorted(sfid.get(f, 0) for f in top))


if __name__ == "__main__":
    main()
