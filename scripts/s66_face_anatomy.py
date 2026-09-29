#!/usr/bin/env python3
"""s66 diag 3: per-face mesh anatomy of BREP#1083.

For each face id in the final mesh: triangle count, vertex count,
boundary loop structure, and cross-face vertex sharing (weld map).
Focus questions:
  - f11/f12 (plane caps): annulus or seam-aliased collapse?
  - f1/f2/f5/f6: which circle edge is the open 31-pt loop?
  - stray long chords on f4/f7/f8/f9/f10.
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

    face_tris = defaultdict(list)
    for ti, fid in fid_of.items():
        face_tris[fid].append(ti)

    # per-face local mesh (using global vertex ids)
    print("fid  ntris  nverts  bnd_edges  closed?")
    for fid in sorted(face_tris):
        tis = face_tris[fid]
        vset = set()
        for ti in tis:
            vset.update(tris[ti])
        # internal edge census WITHIN the face's triangles only
        ec = defaultdict(int)
        for ti in tis:
            a, b, c = tris[ti]
            for u, v in ((a, b), (b, c), (c, a)):
                ec[(min(u, v), max(u, v))] += 1
        bnd = sum(1 for n in ec.values() if n == 1)
        print(f"f{fid:<3} {len(tis):<6} {len(vset):<7} {bnd:<10}"
              f" {'YES' if bnd == 0 else 'no'}")

    # global weld map: vertex id -> set of face ids using it
    v_faces = defaultdict(set)
    for ti, fid in fid_of.items():
        for v in tris[ti]:
            v_faces[v].add(fid)
    shared_v = {v: fs for v, fs in v_faces.items() if len(fs) > 1}
    print(f"\nvertices used by >1 face (welded): {len(shared_v)}")
    pair_hist = defaultdict(int)
    for fs in shared_v.values():
        pair_hist[tuple(sorted(fs))] += 1
    for pair, n in sorted(pair_hist.items(), key=lambda x: -x[1]):
        print(f"  {pair}: {n} verts")

    # coordinate-duplicate census (same coords, different vid)
    coord_v = defaultdict(list)
    for i, p in enumerate(verts):
        coord_v[p].append(i)
    dups = {p: ids for p, ids in coord_v.items() if len(ids) > 1}
    print(f"\ncoordinate-exact duplicate vertices: {len(dups)} groups")
    for p, ids in list(dups.items())[:10]:
        fs = set()
        for i in ids:
            fs |= v_faces[i]
        print(f"  {len(ids)} copies {sorted(fs)} at "
              f"({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})")

    # near-duplicate census (dist < 1e-3): grid hash
    print("\nnear-duplicate pairs (0 < dist <= 2e-3):")
    grid = defaultdict(list)
    for i, p in enumerate(verts):
        key = (int(p[0] * 500), int(p[1] * 500), int(p[2] * 500))
        grid[key].append(i)
    n_near = 0
    for key, ids in grid.items():
        if len(ids) < 2:
            continue
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                i, j = ids[a], ids[b]
                d = math.dist(verts[i], verts[j])
                if 0 < d <= 2e-3:
                    n_near += 1
                    if n_near <= 10:
                        print(f"  d={d:.2e} faces "
                              f"{sorted(v_faces[i])}|{sorted(v_faces[j])}")
    print(f"  total near-dup pairs: {n_near}")

    # f11/f12 vertex layout: radial distribution (annulus check)
    for fid in (11, 12, 3):
        tis = face_tris.get(fid, [])
        if not tis:
            continue
        vset = sorted({v for ti in tis for v in tris[ti]})
        pts = [verts[v] for v in vset]
        print(f"\nf{fid}: {len(tis)} tris, {len(vset)} verts; "
              f"first 6 + last 3:")
        for p in pts[:6]:
            print(f"  ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
        print("  ...")
        for p in pts[-3:]:
            print(f"  ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
        # distance from centroid histogram
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        ds = sorted(math.dist(p, (cx, cy, cz)) for p in pts)
        print(f"  centroid=({cx:.2f},{cy:.2f},{cz:.2f}) "
              f"r: min={ds[0]:.3f} med={ds[len(ds)//2]:.3f} "
              f"max={ds[-1]:.3f}")


if __name__ == "__main__":
    main()
