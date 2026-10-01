#!/usr/bin/env python3
"""s70 diag 1: bnd-edge census of drill HOUSING (BREP#47598) post-s69.

Groups boundary edges by source face + surface type; for Nurbs faces
cross-references the TRI_INPUT dumps (loops, u/v spans, extreme-chain
structure) to design the NURBS few-level lattice detector.
"""
import math, os, re
from collections import defaultdict

OBJ_BASE = "/home/z/my-project/scripts/s70_objs/brep3_HOUSING (BREP#47598)"
TRI_DIR = "/home/z/my-project/scripts/s70_tri"


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
    nfaces = defaultdict(int)
    for fid, cnt in face_bnd.items():
        by_class[stype.get(fid, "?")] += cnt
        nfaces[stype.get(fid, "?")] += 1
    print("\n== bnd by surface class ==")
    for k, v in sorted(by_class.items(), key=lambda kv: -kv[1]):
        print(f"  {k:12s} {v:6d} on {nfaces[k]} faces")
    # Nurbs faces sorted by debt (type string includes params: Nurbs(deg=3/3, cps=4x9))
    nurbs = [(cnt, fid) for fid, cnt in face_bnd.items()
             if stype.get(fid, "").startswith("Nurbs")]
    nurbs.sort(reverse=True)
    print(f"\n== Nurbs debt faces: {len(nurbs)} faces, {sum(c for c, _ in nurbs)} bnd ==")
    for cnt, fid in nurbs[:40]:
        print(f"  fid={fid} step_face={sfid.get(fid)} bnd={cnt}")


def tri_anatomy():
    """Parse TRI_INPUT Nurbs dumps: label, loops, u/v spans, extreme-chain
    decomposition ([flat@vmin][wall][flat@vmax][wall])."""
    print("\n== TRI_INPUT Nurbs anatomy ==")
    files = sorted(os.listdir(TRI_DIR))
    n_qua = 0
    seen_labels = set()
    for fn in files:
        path = os.path.join(TRI_DIR, fn)
        txt = open(path).read()
        head = txt.splitlines()[0]
        m = re.match(r"type=(\S+) forward=(\S+) n_boundary=(\d+) n_holes=(\d+) n_interior=(\d+) n_tris=(\d+) label=(.*)", head)
        if not m:
            continue
        label = m.group(7)
        if "brep47598" not in label:
            continue  # HOUSING only for anatomy
        seen_labels.add(label.split("_Nurbs")[0])
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
        uspan_raw = max(us) - min(us)
        vspan_raw = max(vs) - min(vs)
        # classify: QUAD-like if the chain decomposes as
        # [flat@vmin][wall][flat@vmax][wall] with flat eps = 1e-6*vspan
        flat_eps = 1e-6 * max(vspan_raw, 1e-12)
        vmin, vmax = min(vs), max(vs)
        vspan = max(vmax - vmin, 1e-12)
        n = len(bpts)
        vmin_i = min(range(n), key=lambda k: vs[k])
        vmax_i = max(range(n), key=lambda k: vs[k])
        # flat runs: maximal consecutive runs with v within eps of extreme
        def flat_run(start, step, target):
            # walk around from start in direction step while v ~ target
            cnt, i = 0, start
            while True:
                if abs(vs[i] - target) <= flat_eps:
                    cnt += 1
                    i = (i + step) % n
                    if i == start:
                        break
                else:
                    break
            return cnt, i
        # chain A: vmin_i -> vmax_i (forward), chain B: vmax_i -> vmin_i (forward)
        # flat segments at the extremes of each chain
        a_lo, a_lo_end = flat_run(vmin_i, +1, vmin)
        a_hi, a_hi_end = flat_run(vmax_i, -1, vmax)
        # wall A between a_lo_end and a_hi start
        # (approximate: monotone in v?)
        def seg(i0, i1, step):
            out = [i0]
            i = i0
            while i != i1:
                i = (i + step) % n
                out.append(i)
                if len(out) > n:
                    break
            return out
        # v-mono check for wall A (from end of flat@vmin fwd to start of flat@vmax)
        wall_a = seg(a_lo_end, (vmax_i - a_hi + n) % n and vmax_i, +1) if a_hi else []
        # simpler: walk A from vmin_i to vmax_i fwd, B from vmax_i to vmin_i fwd
        A, B = [], []
        i = vmin_i
        while True:
            A.append(i)
            if i == vmax_i:
                break
            i = (i + 1) % n
        i = vmax_i
        while True:
            B.append(i)
            if i == vmin_i:
                break
            i = (i + 1) % n
        def mono_v(chain):
            return all(vs[chain[i]] <= vs[chain[i + 1]] + 1e-9 for i in range(len(chain) - 1))
        # u-behavior on walls (A interior, B interior): u-const?
        def u_spread(chain, lo, hi):
            uu = [us[k] for k in chain[lo:hi]]
            if not uu:
                return 0.0
            return max(uu) - min(uu)
        wallA_u = u_spread(A, a_lo, len(A) - a_hi)
        wallB_u = u_spread(B, a_hi, len(B) - a_lo)
        quad = (mono_v(A) or mono_v(B)) and (a_lo >= 2 and a_hi >= 2)
        n_qua += 1 if quad else 0
        print(f"{fn}: nb={nb} nh={m.group(4)} ni={m.group(5)} tris={m.group(6)} "
              f"uspan={uspan_raw:.4f} vspan={vspan_raw:.4f} "
              f"flat_lo(A)={a_lo} flat_hi={a_hi} wallA_u={wallA_u:.4f} wallB_u={wallB_u:.4f} "
              f"vmonoA={mono_v(A)} vmonoB={mono_v(B)} "
              f"label={m.group(7)}")
    print(f"\nQUAD-like total: {n_qua}; unique HOUSING Nurbs faces seen: {len(seen_labels)}")




def joined():
    """Join per-face bnd debt with TRI anatomy, keyed by internal fid."""
    verts, tris, fid_of, stype, sfid = load_obj(OBJ_BASE)
    edge_tris = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for u, v in ((a, b), (b, c), (c, a)):
            e = (min(u, v), max(u, v))
            edge_tris[e].append(ti)
    bnd = [e for e, ts in edge_tris.items() if len(ts) == 1]
    face_bnd = defaultdict(int)
    for e in bnd:
        face_bnd[fid_of.get(edge_tris[e][0], -1)] += 1
    # parse anatomy by fid
    anat = {}
    for fn in sorted(os.listdir(TRI_DIR)):
        txt = open(os.path.join(TRI_DIR, fn)).read()
        head = txt.splitlines()[0]
        m = re.match(r"type=(\S+) forward=(\S+) n_boundary=(\d+) n_holes=(\d+) n_interior=(\d+) n_tris=(\d+) label=(.*)", head)
        if not m:
            continue
        label = m.group(7)
        if "brep47598" not in label:
            continue
        fmn = re.match(r"brep\d+_f(\d+)_", label)
        if not fmn:
            continue
        fid = int(fmn.group(1))
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
        vs = [p[1] for p in bpts]
        us = [p[0] for p in bpts]
        n = len(bpts)
        vmin, vmax = min(vs), max(vs)
        vspan = max(vmax - vmin, 1e-12)
        flat_eps = 1e-6 * vspan
        vmin_i = min(range(n), key=lambda k: vs[k])
        vmax_i = max(range(n), key=lambda k: vs[k])
        def flat_run(start, step, target):
            cnt, i = 0, start
            while True:
                if abs(vs[i] - target) <= flat_eps:
                    cnt += 1
                    i = (i + step) % n
                    if i == start:
                        break
                else:
                    break
            return cnt
        a_lo = flat_run(vmin_i, +1, vmin)
        a_hi = flat_run(vmax_i, -1, vmax)
        b_hi = flat_run(vmax_i, +1, vmax)
        b_lo = flat_run(vmin_i, -1, vmin)
        A = []
        i = vmin_i
        while True:
            A.append(i)
            if i == vmax_i:
                break
            i = (i + 1) % n
        B = []
        i = vmax_i
        while True:
            B.append(i)
            if i == vmin_i:
                break
            i = (i + 1) % n
        def mono(chain):
            return all(vs[chain[i]] <= vs[chain[i + 1]] + 1e-9 for i in range(len(chain) - 1))
        def usp(chain, lo, hi):
            uu = [us[k] for k in chain[lo:hi]]
            return (max(uu) - min(uu)) if uu else 0.0
        anat[fid] = dict(
            nb=nb, nh=int(m.group(4)), ni=int(m.group(5)),
            tris=int(m.group(6)),
            uspan=max(us) - min(us), vspan=vspan,
            a_lo=a_lo, a_hi=a_hi, b_hi=b_hi, b_lo=b_lo,
            wallA_u=usp(A, a_lo, len(A) - a_hi),
            wallB_u=usp(B, b_hi, len(B) - b_lo),
            vmonoA=mono(A), vmonoB=mono(B),
        )
    print("\n== JOINED table: fid bnd nb nh ni | vspan uspan | flat(lo/hi A, hi/lo B) | wallA_u wallB_u | monoA/B ==")
    rows = []
    nurbs_fids = [f for f in face_bnd if stype.get(f, "").startswith("Nurbs")]
    for fid in sorted(nurbs_fids):
        a = anat.get(fid)
        if a is None:
            rows.append((face_bnd[fid], fid, None))
        else:
            rows.append((face_bnd[fid], fid, a))
    rows.sort(reverse=True)
    tot = 0
    for cnt, fid, a in rows:
        tot += cnt
        if a is None:
            print(f"  fid={fid:4d} bnd={cnt:4d} (no tri dump)")
            continue
        quadlike = (a["a_lo"] >= 8 and a["a_hi"] >= 8) or (a["b_lo"] >= 8 and a["b_hi"] >= 8)
        walls_const = a["wallA_u"] < 5e-3 and a["wallB_u"] < 5e-3
        cls = ("QUAD" if quadlike and walls_const else
               "FLAT" if quadlike else
               "LUNE" if (a["vmonoA"] or a["vmonoB"]) else "?")
        print(f"  fid={fid:4d} bnd={cnt:4d} nb={a['nb']:4d} nh={a['nh']} ni={a['ni']:3d} "
              f"vspan={a['vspan']:.3f} uspan={a['uspan']:.3f} "
              f"flatA={a['a_lo']}/{a['a_hi']} flatB={a['b_hi']}/{a['b_lo']} "
              f"wallA_u={a['wallA_u']:.4f} wallB_u={a['wallB_u']:.4f} "
              f"monoA={a['vmonoA']} monoB={a['vmonoB']} cls={cls}")
    print(f"  TOTAL joined: {tot} bnd over {len(rows)} Nurbs faces")


if __name__ == "__main__":
    census()
    tri_anatomy()
    joined()
