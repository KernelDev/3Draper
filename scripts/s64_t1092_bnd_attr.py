#!/usr/bin/env python3
"""s64 diag: attribute #1092 TRANSPORTROLLE boundary edges to faces + geometry.

Loads the FINAL OBJ dump (brep26) + fmap, computes boundary edges
(edges used by exactly 1 triangle), attributes each to its owning face
(via fmap), then for every boundary vertex checks which OTHER faces have
a vertex nearby (radius ~0.05) — distinguishing MIS-STITCH (neighbor
geometry exists but not welded) from ORPHAN (geometry missing).
"""
import sys
from collections import defaultdict

OBJ = "/tmp/s64_objs/brep26_TRANSPORTROLLE (BREP#1092).obj"
FMAP = "/tmp/s64_objs/brep26_TRANSPORTROLLE (BREP#1092).fmap"

def main():
    verts = []
    tris = []
    for line in open(OBJ):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(p) - 1 for p in line.split()[1:4])
            tris.append((a, b, c))
    fid_of = {}
    for line in open(FMAP):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of[int(ti)] = int(fid)
    print(f"verts={len(verts)} tris={len(tris)}")

    edge_count = defaultdict(int)
    edge_faces = defaultdict(set)
    for ti, (a, b, c) in enumerate(tris):
        f = fid_of.get(ti, -1)
        for u, v in ((a, b), (b, c), (c, a)):
            e = (u, v) if u < v else (v, u)
            edge_count[e] += 1
            edge_faces[e].add(f)

    bnd = [e for e, n in edge_count.items() if n == 1]
    nm = [e for e, n in edge_count.items() if n > 2]
    print(f"boundary edges={len(bnd)} non-manifold={len(nm)}")

    # attribute to faces
    bnd_by_face = defaultdict(list)
    for e in bnd:
        bnd_by_face[list(edge_faces[e])[0]].append(e)
    print("\nboundary edges by face:")
    for f in sorted(bnd_by_face, key=lambda f: -len(bnd_by_face[f])):
        print(f"  f{f}: {len(bnd_by_face[f])}")

    # per-boundary-vertex neighbor census: which faces have vertices within r
    # vertex -> faces that use it
    vert_faces = defaultdict(set)
    for ti, (a, b, c) in enumerate(tris):
        f = fid_of.get(ti, -1)
        for v in (a, b, c):
            vert_faces[v].add(f)
    # grid for radius search
    cell = 0.25
    grid = defaultdict(list)
    for i, (x, y, z) in enumerate(verts):
        grid[(int(x // cell), int(y // cell), int(z // cell))].append(i)

    def near(i, r):
        x, y, z = verts[i]
        out = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for j in grid[(int(x // cell) + dx, int(y // cell) + dy, int(z // cell) + dz)]:
                        if j == i:
                            continue
                        X, Y, Z = verts[j]
                        d2 = (X - x) ** 2 + (Y - y) ** 2 + (Z - z) ** 2
                        if d2 <= r * r:
                            out.append((j, d2 ** 0.5))
        return out

    bnd_verts = set()
    for e in bnd:
        bnd_verts.add(e[0]); bnd_verts.add(e[1])
    print(f"\nboundary vertices: {len(bnd_verts)}")

    # classify each boundary edge of the TOP faces f29/f32
    for face in (29, 32, 3, 12):
        edges = bnd_by_face.get(face, [])
        if not edges:
            continue
        # STEP-edge grouping: consecutive bnd edges sharing vertices form chains
        adj = defaultdict(list)
        for e in edges:
            adj[e[0]].append(e[1]); adj[e[1]].append(e[0])
        chains = []
        seen = set()
        for e in edges:
            for s in e:
                if s in seen:
                    continue
                # walk
                comp, stack = [], [s]
                while stack:
                    n = stack.pop()
                    if n in seen:
                        continue
                    seen.add(n); comp.append(n)
                    stack.extend(adj[n])
                chains.append(comp)
        lens = sorted(len(c) for c in chains)
        # neighbor census per chain endpoint sample
        print(f"\n== f{face}: {len(edges)} bnd edges, {len(chains)} chains (sizes {lens})")
        for ci, comp in enumerate(sorted(chains, key=len, reverse=True)[:6]):
            vs = comp
            # census of nearby other-face vertices over chain
            nb_faces = defaultdict(int)
            orphan = 0
            for v in vs:
                found = False
                for (j, d) in near(v, 0.05):
                    for f2 in vert_faces[j]:
                        if f2 != face:
                            nb_faces[f2] += 1
                            found = True
                if not found:
                    orphan += 1
            # chain geometry extent
            xs = [verts[v][0] for v in vs]; ys = [verts[v][1] for v in vs]; zs = [verts[v][2] for v in vs]
            print(f"  chain{ci}: {len(vs)} verts, orphan={orphan}/{len(vs)}, "
                  f"nb_faces={dict(sorted(nb_faces.items(), key=lambda kv: -kv[1])[:4])}, "
                  f"bbox=[{min(xs):.3f},{max(xs):.3f}]x[{min(ys):.3f},{max(ys):.3f}]x[{min(zs):.3f},{max(zs):.3f}]")

if __name__ == "__main__":
    main()
