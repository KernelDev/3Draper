#!/usr/bin/env python3
"""s130 п.2: rim-vs-slit classification of f65 (#27977) once-edges.

Rim = consecutive pairs of the face boundary polygon (TRI_INPUT 'b'
lines, first-pass call tri_0043, n_boundary=279). Slit = once-edges that
are NOT rim (created by Step-6 refinement). Matches by distance (dump
prints 6 decimals, tri file carries full precision).
"""
import math, os, sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RIM_SRC = f"{BASE}/forensics/s130/fm27977/tri/tri_0043_Torus.txt"
DUMPS = f"{BASE}/forensics/s130/refiters"


def load_rim():
    rim = []
    for ln in open(RIM_SRC, errors="replace"):
        if ln.startswith("b "):
            f = ln.split()
            rim.append((float(f[3]), float(f[4]), float(f[5])))
    return rim


def once_edges(path):
    out = []
    for ln in open(path, errors="replace"):
        if not ln.startswith("once "):
            continue
        pos = ln.split("pos=")[1].split(" tris=")[0]
        pa, pb = pos.split("/")
        pa = tuple(float(x) for x in pa.strip("()").split(","))
        pb = tuple(float(x) for x in pb.strip("()").split(","))
        tris = ln.split("tris=")[1].strip()
        out.append((pa, pb, tris))
    return out


def near(p, q, tol=3e-4):
    return math.dist(p, q) < tol


def main():
    rim = load_rim()
    n = len(rim)
    rim_segs = [(rim[i], rim[(i + 1) % n]) for i in range(n)]
    print(f"rim points: {n}")

    files = [
        ("p0 iter0 (t691)", "brep27977_f65_Torus_iter0_t691.txt"),
        ("p0 iter1 (t705 = SHIPS)", "brep27977_f65_Torus_iter1_t705.txt"),
    ]
    for tag, fn in files:
        es = once_edges(f"{DUMPS}/{fn}")
        slit, rim_n = [], 0
        for pa, pb, tris in es:
            is_rim = any((near(pa, a) and near(pb, b)) or
                         (near(pb, a) and near(pa, b))
                         for a, b in rim_segs)
            if is_rim:
                rim_n += 1
            else:
                slit.append((pa, pb, tris))
        print(f"\n{tag}: once={len(es)} rim={rim_n} SLIT={len(slit)}")
        for pa, pb, tris in slit:
            print(f"  SLIT len={math.dist(pa, pb):.5f} "
                  f"{pa} -- {pb} {tris}")


if __name__ == "__main__":
    main()
