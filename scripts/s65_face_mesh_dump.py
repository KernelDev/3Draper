#!/usr/bin/env python3
"""s65 diag 5: full mesh dump of f9 (flap cone), f16 (skirt), f1-fan region
for #1086. Reconstructs each face's boundary polygon (boundary of the
face-triangle subset) and checks: does f16's boundary contain the ARC
chain (local y=5.0, r=2.8868) instead of the lip bspline (y in [4.79,5])?
Does f9's polygon have both chains or a collapsed one?
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

def face_boundary(vs, tris, fids, face):
    """Boundary edges of one face's triangle subset."""
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        if fids.get(ti) != face:
            continue
        for u, v in ((a, b), (b, c), (c, a)):
            e = (u, v) if u < v else (v, u)
            e2t[e].append(ti)
    return [e for e, l in e2t.items() if len(l) == 1], e2t

def local_coords(p):
    """Part-local coords: world_x = -45 + local_y; local_x = -660.6873 - world_y;
    local_z = world_z - 520.5.  (derived from corner mapping)"""
    ly = p[0] + 45.0
    lx = -660.6873 - p[1]
    lz = p[2] - 520.5
    return (lx, ly, lz)

def main():
    vs, tris, fids = load(OBJ)

    for face in (9, 16, 1):
        bnd, e2t = face_boundary(vs, tris, fids, face)
        # vertices on the face boundary
        bverts = sorted({v for e in bnd for v in e})
        print(f"\n=== f{face}: boundary verts={len(bverts)} bnd edges={len(bnd)} ===")
        # classify boundary verts by local y and r
        levels = defaultdict(list)
        for v in bverts:
            lx, ly, lz = local_coords(vs[v])
            r = math.hypot(lx, lz)
            levels[round(ly, 2)].append((v, r))
        for ly in sorted(levels):
            rs = [r for _, r in levels[ly]]
            print(f"  local_y={ly:.2f}: {len(rs)} verts, r in [{min(rs):.4f}, {max(rs):.4f}]")
        # sample the actual boundary chain order for the longest chain
        adj = defaultdict(list)
        for e in bnd:
            adj[e[0]].append(e[1])
            adj[e[1]].append(e[0])
        # walk chains
        chains = []
        visited = set()
        for v in bverts:
            if v in visited or len(adj[v]) != 2:
                continue
            chain = [v]
            visited.add(v)
            prev = None
            cur = v
            while True:
                nxts = [n for n in adj[cur] if n != prev]
                if not nxts:
                    break
                nxt = nxts[0]
                if nxt in visited and nxt != chain[0]:
                    break
                chain.append(nxt)
                visited.add(nxt)
                prev, cur = cur, nxt
                if cur == v:
                    break
            chains.append(chain)
        print(f"  closed chains: {len(chains)}")
        for i, ch in enumerate(chains):
            print(f"  chain[{i}]: {len(ch)} verts, first/last 3 local coords:")
            for v in ch[:3] + ch[-3:]:
                lx, ly, lz = local_coords(vs[v])
                print(f"    v{v}: local=({lx:.4f}, {ly:.4f}, {lz:.4f}) r={math.hypot(lx,lz):.4f}")
        # junction verts (valence != 2)
        junc = [v for v in bverts if len(adj[v]) != 2]
        if junc:
            print(f"  junction verts (valence!=2): {len(junc)}")
            for v in junc[:8]:
                lx, ly, lz = local_coords(vs[v])
                print(f"    v{v}: val={len(adj[v])} local=({lx:.4f}, {ly:.4f}, {lz:.4f}) r={math.hypot(lx,lz):.4f}")

    # f9 interior check: all f9 triangle apexes
    print("\n=== f9 all vertices (local) ===")
    f9v = sorted({v for ti, f in fids.items() if f == 9 for v in tris[ti]})
    for v in f9v:
        lx, ly, lz = local_coords(vs[v])
        print(f"  v{v}: local=({lx:.4f}, {ly:.4f}, {lz:.4f}) r={math.hypot(lx,lz):.4f}")

if __name__ == "__main__":
    main()
