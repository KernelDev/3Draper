#!/usr/bin/env python3
"""s69 diag 1: bnd-edge census of drill HOUSING (BREP#47598) post-s68.

Groups boundary edges by source face + surface type; for Torus faces
cross-references the TRI_INPUT dumps (u/v spans, loop count) to design
the TORUS fillet band detector.
"""
import math, os, re
from collections import defaultdict

OBJ_BASE = "/home/z/my-project/scripts/s69_objs/brep3_HOUSING (BREP#47598)"
TRI_DIR = "/home/z/my-project/scripts/s69_tri"


def load_obj(base):
    verts, tris = [], []
    for line in open(base + ".obj"):
        if line.startswith("v "):
            _, x, y, z = line.split()
            verts.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = (int(t) - 1 for t in line.split()[1:4])
            tris.append((a, b, c))
    fid_of = {}
    for line in open(base + ".fmap"):
        if line.startswith("t "):
            _, ti, fid = line.split()
            fid_of[int(ti)] = int(fid)
    stype, sfid = {}, {}
    for line in open(base + ".facemap"):
        if line.startswith("f "):
            parts = line.split()
            fid = int(parts[1])
            st = " ".join(parts[2:-2])
            sfid[fid] = int(parts[-2])
            stype[fid] = st
    return verts, tris, fid_of, stype, sfid


def census():
    verts, tris, fid_of, stype, sfid = load_obj(OBJ_BASE)
    print(f"mesh: {len(verts)} verts, {len(tris)} tris")
    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
    bnd = [e for e, ts in edge_tris.items() if len(ts) == 1]
    print(f"total bnd: {len(bnd)}")
    face_bnd = defaultdict(int)
    for e in bnd:
        face_bnd[fid_of.get(edge_tris[e][0], -1)] += 1
    by_class = defaultdict(int)
    for fid, cnt in face_bnd.items():
        by_class[stype.get(fid, "?")] += cnt
    print("\n== bnd by surface class ==")
    for k, v in sorted(by_class.items(), key=lambda kv: -kv[1]):
        print(f"  {k:12s} {v}")
    # Torus faces sorted by debt
    torus = [(cnt, fid) for fid, cnt in face_bnd.items() if stype.get(fid) == "Torus"]
    torus.sort(reverse=True)
    print(f"\n== Torus debt faces: {len(torus)} faces, {sum(c for c, _ in torus)} bnd ==")
    for cnt, fid in torus[:30]:
        print(f"  fid={fid} step_face={sfid.get(fid)} bnd={cnt}")


def tri_anatomy():
    """Parse TRI_INPUT Torus dumps: label, loops, u/v spans, v-extreme split."""
    print("\n== TRI_INPUT Torus anatomy ==")
    files = sorted(os.listdir(TRI_DIR))
    for fn in files:
        path = os.path.join(TRI_DIR, fn)
        txt = open(path).read()
        head = txt.splitlines()[0]
        m = re.match(r"type=(\S+) forward=(\S+) n_boundary=(\d+) n_holes=(\d+) n_interior=(\d+) n_tris=(\d+) label=(.*)", head)
        if not m:
            continue
        nb = int(m.group(3))
        bpts = []
        for line in txt.splitlines():
            if line.startswith("b "):
                _, u, v = line.split()
                bpts.append((float(u), float(v)))
            elif line.startswith("h ") or line.startswith("i "):
                break
        if len(bpts) != nb:
            continue
        us = [p[0] for p in bpts]
        vs = [p[1] for p in bpts]
        # seam-normalized spans (unwrap along the walk)
        def unwrap(xs, period=2 * math.pi):
            out = [xs[0]]
            for x in xs[1:]:
                prev = out[-1]
                while x - prev > period / 2:
                    x -= period
                while prev - x > period / 2:
                    x += period
                out.append(x)
            return out
        uu = unwrap(us)
        vv = unwrap(vs)
        uspan = max(uu) - min(uu)
        vspan = max(vv) - min(vv)
        # v-extreme split: chains from vmin to vmax
        n = len(bpts)
        vmin_i = min(range(n), key=lambda k: vv[k])
        vmax_i = max(range(n), key=lambda k: vv[k])
        # monotone check on both walks
        def mono(chain):
            return all(vv[chain[i]] <= vv[chain[i + 1]] + 1e-9 for i in range(len(chain) - 1))
        a = []
        i = vmin_i
        while True:
            a.append(i)
            if i == vmax_i:
                break
            i = (i + 1) % n
        b = []
        i = vmax_i
        while True:
            b.append(i)
            if i == vmin_i:
                break
            i = (i + 1) % n
        b = b[::-1]
        print(f"{fn}: nb={nb} nh={m.group(4)} ni={m.group(5)} tris={m.group(6)} "
              f"uspan={uspan:.3f} vspan={vspan:.3f} "
              f"vmonoA={mono(a)}({len(a)}) vmonoB={mono(b)}({len(b)}) "
              f"label={m.group(7)}")


if __name__ == "__main__":
    census()
    tri_anatomy()
