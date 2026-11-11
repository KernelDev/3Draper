#!/usr/bin/env python3
"""s130: MERGED-level forensics of #27977 REAL pairs f(7,7)/f(65,65)
(Spit, BREP#27977, brep_idx=7, INST=identity).

s129 proved the pairs are emission-invariant (REFINE=0 / SKIP_RESCUED=1
leave census unchanged) => they live on the MERGED level. This script
tracks them through the per-BREP repair chain stage dumps
(DRAPPER_DUMP_STAGE_OBJS=forensics/s130/stages_27977):

  per pass p0/p1 (gated retry), code order (converter.rs line nos.):
    b27977-d-after-merge   (6766)
    b27977-d-after-weld    (6847)
    b27977-d-after-tj      (6886)
    b27977-d-after-gapfill (6913)
    b27977-d-after-winding (~6920+)
    d-after-flapclean       (7137)
    d-after-bornfold-split  (7241)
    d-after-deflip          (7278, conditional)
    d-after-postweldwind    (17593)

Pair identification (replicates fold_face_probe scan):
  ang = degrees(acos(dot(n0,n1))) > 170  (normals from winding)
  topo_consistent = edge traversed in opposite directions
  same_side = apexes on the same side of the edge line
  h0/h1 = apex distance to the shared edge LINE
  FOLD-OVER = topo_consistent + same_side
Match key vs census: h within +-8%, ang within 1.0 deg, class FOLD-OVER.
"""
import os, re, sys, math
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DUMP = f"{BASE}/forensics/s130/stages_27977"
CENSUS = f"{BASE}/forensics/s130/fp7_spit.txt"

STAGES = [
    "b27977-d-after-merge",
    "b27977-d-after-weld",
    "b27977-d-after-tj",
    "b27977-d-after-gapfill",
    "b27977-d-after-winding",
    "d-after-flapclean",
    "d-after-bornfold-split",
    "d-after-deflip",
    "d-after-postweldwind",
]
PASSES = ["p0", "p1"]


def load_census_pairs():
    """REAL [FOLD-OVERFAT] pairs from the isolated target-7 run."""
    pairs = []
    for ln in open(CENSUS, errors="replace"):
        if not ln.startswith("[FOLD-OVERFAT]"):
            continue
        if "BREP#27977" not in ln:
            continue
        m = re.search(r"faces=\((\d+),(\d+)\).*?tris=\(\[([^]]*)\],\[([^]]*)\]\)"
                      r".*?h=\(([\d.]+),([\d.]+)\).*?mid=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", ln)
        if not m:
            continue
        t0 = [int(x) for x in m.group(3).split(",")]
        t1 = [int(x) for x in m.group(4).split(",")]
        pairs.append({
            "face": int(m.group(1)),
            "t0": t0, "t1": t1,
            "h": (float(m.group(5)), float(m.group(6))),
            "ang": float(re.search(r"ang=([\d.]+)", ln).group(1)),
            "mid": (float(m.group(7)), float(m.group(8)), float(m.group(9))),
            "raw": ln.rstrip(),
        })
    return pairs


def load_obj(path):
    if not os.path.exists(path):
        return None
    verts, tris = [], []
    for ln in open(path, errors="replace"):
        if ln.startswith("v "):
            _, x, y, z = ln.split()
            verts.append((float(x), float(y), float(z)))
        elif ln.startswith("f "):
            _, i, j, k = ln.split()
            tris.append((int(i) - 1, int(j) - 1, int(k) - 1))  # 1-based -> 0
    return verts, tris


def load_fmap(path):
    if not os.path.exists(path):
        return {}
    fm = {}
    for ln in open(path, errors="replace"):
        if ln.startswith("t "):
            _, ti, fid = ln.split()
            fm[int(ti)] = int(fid)
    return fm


def unit_normal(verts, tri):
    (a, b, c) = (verts[tri[0]], verts[tri[1]], verts[tri[2]])
    u = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    v = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
         u[0] * v[1] - u[1] * v[0])
    l = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
    return (n[0] / l, n[1] / l, n[2] / l) if l > 1e-30 else None


def dist(p, q):
    return math.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2 + (p[2] - q[2])**2)


def dist_line(p, a, b):
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ap = (p[0] - a[0], p[1] - a[1], p[2] - a[2])
    cr = (ap[1] * ab[2] - ap[2] * ab[1], ap[2] * ab[0] - ap[0] * ab[2],
          ap[0] * ab[1] - ap[1] * ab[0])
    l = math.sqrt(cr[0]**2 + cr[1]**2 + cr[2]**2)
    base = math.sqrt(ab[0]**2 + ab[1]**2 + ab[2]**2)
    return l / max(base, 1e-30)


def scan_pairs(verts, tris, fm, mid, radius=0.6):
    """All >170deg interior-edge pairs with edge midpoint near `mid`."""
    edge_mid = defaultdict(list)  # (i,j) sorted -> [(ti, pos_in_tri)]
    for ti, t in enumerate(tris):
        for k in range(3):
            i, j = t[k], t[(k + 1) % 3]
            edge_mid[(min(i, j), max(i, j))].append((ti, k))
    out = []
    for (i, j), users in edge_mid.items():
        if len(users) != 2:
            continue
        a, b = verts[i], verts[j]
        em = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2)
        if dist(em, mid) > radius:
            continue
        (t0, p0), (t1, p1) = users[0], users[1]
        n0, n1 = unit_normal(verts, tris[t0]), unit_normal(verts, tris[t1])
        if not n0 or not n1:
            continue
        dot = n0[0]*n1[0] + n0[1]*n1[1] + n0[2]*n1[2]
        ang = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        if ang <= 170.0:
            continue
        # topo: edge (i,j) in t0 at pos p0=(k): t0 goes ...i->j... if
        # tris[t0][p0]==i. Opposite directions <=> t0 has i->j AND t1 has
        # j->i (or vice versa).
        d0 = tris[t0][p0], tris[t0][(p0 + 1) % 3]
        d1 = tris[t1][p1], tris[t1][(p1 + 1) % 3]
        topo_consistent = (d0[0] == d1[1] and d0[1] == d1[0])
        apex_of = lambda t, e0, e1: next(verts[v] for v in t if v != e0 and v != e1)
        c0 = apex_of(tris[t0], i, j)
        c1 = apex_of(tris[t1], i, j)
        e = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
        side = lambda q: (
            e[1]*(q[2]-a[2]) - e[2]*(q[1]-a[1]),
            e[2]*(q[0]-a[0]) - e[0]*(q[2]-a[2]),
            e[0]*(q[1]-a[1]) - e[1]*(q[0]-a[0]))
        s0, s1 = side(c0), side(c1)
        same_side = s0[0]*s1[0] + s0[1]*s1[1] + s0[2]*s1[2] > 0
        cls = ("FOLD-OVER" if topo_consistent else
               ("DOUBLE-BROKEN" if same_side else "WINDING-FLIP")) if same_side \
            else ("CURVED-180" if topo_consistent else "WINDING-FLIP")
        # (classify exactly like probe: topo_consistent? same_side? ...)
        if topo_consistent:
            cls = "FOLD-OVER" if same_side else "CURVED-180"
        else:
            cls = "DOUBLE-BROKEN" if same_side else "WINDING-FLIP"
        out.append({
            "edge": (i, j), "t0": t0, "t1": t1,
            "fid0": fm.get(t0, 0), "fid1": fm.get(t1, 0),
            "ang": ang, "cls": cls, "topo": topo_consistent,
            "same_side": same_side,
            "h": (dist_line(c0, a, b), dist_line(c1, a, b)),
            "mid": em, "edge_pos": (a, b),
        })
    return out


def matches_census(c, pair):
    if c["cls"] not in ("FOLD-OVER", "DOUBLE-BROKEN"):
        return False
    hr, hs = pair["h"]
    if abs(c["h"][0] - hr) / hr > 0.08 or abs(c["h"][1] - hs) / hs > 0.08:
        return False
    if abs(c["ang"] - pair["ang"]) > 1.0:
        return False
    return True


def main():
    pairs = load_census_pairs()
    print(f"census REAL pairs: {[(p['face'], p['h'], p['ang']) for p in pairs]}")
    for pair in pairs:
        print(f"\n=== pair f({pair['face']},{pair['face']}) "
              f"h={pair['h']} ang={pair['ang']} mid={pair['mid']} "
              f"tris={pair['t0']}/{pair['t1']}")

        # -- shipped-pass determination: last stage containing census
        #    tri indices with census geometry.
        shipped = None
        for p in PASSES:
            st = f"{p}_d-after-postweldwind"
            data = load_obj(f"{DUMP}/{st}.obj")
            if not data:
                continue
            verts, tris = data
            n = max(pair["t0"] + pair["t1"])
            if n >= len(verts):
                continue
            # positions of census indices -> geometry check
            t0 = pair["t0"]; t1 = pair["t1"]
            a = verts[t0[1]]; b = verts[t0[2]]  # shared edge guess: common verts
            common = [v for v in t0 if v in t1]
            if len(common) != 2:
                continue
            a, b = verts[common[0]], verts[common[1]]
            c0 = next(verts[v] for v in t0 if v not in common)
            c1 = next(verts[v] for v in t1 if v not in common)
            h = (dist_line(c0, a, b), dist_line(c1, a, b))
            ok_h = (abs(h[0]-pair["h"][0])/pair["h"][0] < 0.02 and
                    abs(h[1]-pair["h"][1])/pair["h"][1] < 0.02)
            if ok_h:
                shipped = p
                print(f"  shipped pass = {p} "
                      f"(postweldwind h={h} matches census {pair['h']})")
                break
        if not shipped:
            print("  WARNING: shipped pass not determined; assuming p1")

        # -- per-stage scan (both passes, code order)
        for p in PASSES:
            first_seen = None
            for st in STAGES:
                path = f"{DUMP}/{p}_{st}.obj"
                data = load_obj(path)
                if data is None:
                    continue
                verts, tris = data
                fm = load_fmap(f"{DUMP}/{p}_{st}.fmap")
                cands = scan_pairs(verts, tris, fm, pair["mid"])
                hits = [c for c in cands if matches_census(c, pair)]
                tag = ""
                if hits:
                    if first_seen is None:
                        first_seen = st
                        tag = "  <== BORN HERE"
                    c = hits[0]
                    print(f"  {p} {st:28s} PAIR "
                          f"e={c['edge']} tris={c['t0']}/{c['t1']} "
                          f"fids={c['fid0']}/{c['fid1']} "
                          f"h=({c['h'][0]:.4f},{c['h'][1]:.4f}) "
                          f"ang={c['ang']:.2f} cls={c['cls']}{tag}")
                elif cands:
                    c0 = cands[0]
                    print(f"  {p} {st:28s} near-pairs={len(cands)} "
                          f"(no census match) e.g. "
                          f"h=({c0['h'][0]:.4f},{c0['h'][1]:.4f}) "
                          f"ang={c0['ang']:.2f} "
                          f"cls={c0['cls']} fids="
                          f"{c0['fid0']}/{c0['fid1']}")
                else:
                    print(f"  {p} {st:28s} no >170deg pair near mid")
            if first_seen:
                print(f"  => {p}: pair BORN at {first_seen}")


if __name__ == "__main__":
    main()
