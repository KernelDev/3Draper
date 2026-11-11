#!/usr/bin/env python3
"""s130: do the #27977 REAL pairs exist in the per-face EMISSIONS?

If yes -> EMISSION-BORN for the face itself (s129 'MERGED-level' claim
needs revision: the invariance to REFINE/SKIP must come from elsewhere).
If no  -> merge-SNAPPED into existence (positions moved by merge dedup).

Emission files: forensics/s130/fm27977/mesh_brep27977_f{fid}_{Type}_a{att}.txt
Format: "v x y z" (0-based t references) + "t i j k".
Attempts: f7 a6/a79, f65 a64/a137 (a<20 first pass, a>=20 retry).
INST for #27977 is identity -> same frame as census.
"""
import os, re, math
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EMDIR = f"{BASE}/forensics/s130/fm27977"
CENSUS = f"{BASE}/forensics/s130/fp7_spit.txt"

TARGETS = [
    {"face": 7, "files": ["mesh_brep27977_f7_Torus_a6.txt",
                          "mesh_brep27977_f7_Torus_a79.txt"]},
    {"face": 65, "files": ["mesh_brep27977_f65_Torus_a64.txt",
                           "mesh_brep27977_f65_Torus_a137.txt"]},
]


def load_census_pairs():
    pairs = []
    for ln in open(CENSUS, errors="replace"):
        if not ln.startswith("[FOLD-OVERFAT]") or "BREP#27977" not in ln:
            continue
        m = re.search(
            r"faces=\((\d+),(\d+)\).*?tris=\(\[([^]]*)\],\[([^]]*)\]\)"
            r".*?h=\(([\d.]+),([\d.]+)\).*?mid=\(([-\d.]+),([-\d.]+),([-\d.]+)\)", ln)
        t0 = [int(x) for x in m.group(3).split(",")]
        t1 = [int(x) for x in m.group(4).split(",")]
        pairs.append({"face": int(m.group(1)), "t0": t0, "t1": t1,
                      "h": (float(m.group(5)), float(m.group(6))),
                      "ang": float(re.search(r"ang=([\d.]+)", ln).group(1)),
                      "mid": (float(m.group(7)), float(m.group(8)),
                              float(m.group(9)))})
    return pairs


def load_em(path):
    verts, tris = [], []
    for ln in open(path, errors="replace"):
        if ln.startswith("#"):
            continue
        f = ln.split()
        if not f:
            continue
        if f[0] == "v":
            verts.append((float(f[2]), float(f[3]), float(f[4])))
        elif f[0] == "t":
            tris.append((int(f[2]), int(f[3]), int(f[4])))
    return verts, tris


def dist(p, q):
    return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2)


def dist_line(p, a, b):
    ab = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
    ap = (p[0]-a[0], p[1]-a[1], p[2]-a[2])
    cr = (ap[1]*ab[2]-ap[2]*ab[1], ap[2]*ab[0]-ap[0]*ab[2],
          ap[0]*ab[1]-ap[1]*ab[0])
    return math.sqrt(cr[0]**2+cr[1]**2+cr[2]**2) / max(
        math.sqrt(ab[0]**2+ab[1]**2+ab[2]**2), 1e-30)


def unit_normal(verts, tri):
    a, b, c = verts[tri[0]], verts[tri[1]], verts[tri[2]]
    u = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
    v = (c[0]-a[0], c[1]-a[1], c[2]-a[2])
    n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
    l = math.sqrt(n[0]**2+n[1]**2+n[2]**2)
    return (n[0]/l, n[1]/l, n[2]/l) if l > 1e-30 else None


def scan(verts, tris, pair, radius=0.6):
    mid = pair["mid"]
    edge_users = defaultdict(list)
    for ti, t in enumerate(tris):
        for k in range(3):
            i, j = t[k], t[(k+1) % 3]
            edge_users[(min(i, j), max(i, j))].append((ti, k))
    hits, near = [], []
    for (i, j), users in edge_users.items():
        if len(users) != 2:
            continue
        a, b = verts[i], verts[j]
        em = ((a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2)
        if dist(em, mid) > radius:
            continue
        (t0, p0), (t1, p1) = users[0], users[1]
        n0, n1 = unit_normal(verts, tris[t0]), unit_normal(verts, tris[t1])
        if not n0 or not n1:
            continue
        ang = math.degrees(math.acos(max(-1, min(1,
            n0[0]*n1[0]+n0[1]*n1[1]+n0[2]*n1[2]))))
        if ang <= 170:
            continue
        d0 = tris[t0][p0], tris[t0][(p0+1) % 3]
        d1 = tris[t1][p1], tris[t1][(p1+1) % 3]
        topo = d0[0] == d1[1] and d0[1] == d1[0]
        apex = lambda t: next(verts[v] for v in t if v != i and v != j)
        c0, c1 = apex(tris[t0]), apex(tris[t1])
        e = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
        sd = lambda q: (e[1]*(q[2]-a[2])-e[2]*(q[1]-a[1]),
                        e[2]*(q[0]-a[0])-e[0]*(q[2]-a[2]),
                        e[0]*(q[1]-a[1])-e[1]*(q[0]-a[0]))
        s0, s1 = sd(c0), sd(c1)
        same = s0[0]*s1[0]+s0[1]*s1[1]+s0[2]*s1[2] > 0
        cls = ("FOLD-OVER" if same else "CURVED-180") if topo else \
              ("DOUBLE-BROKEN" if same else "WINDING-FLIP")
        h = (dist_line(c0, a, b), dist_line(c1, a, b))
        rec = {"edge": (i, j), "tris": (t0, t1), "ang": ang, "cls": cls,
               "h": h, "mid": em,
               "verts6": (verts[i], verts[j], c0, c1)}
        near.append(rec)
        if cls in ("FOLD-OVER", "DOUBLE-BROKEN"):
            if (abs(h[0]-pair["h"][0])/pair["h"][0] < 0.08 and
                    abs(h[1]-pair["h"][1])/pair["h"][1] < 0.08 and
                    abs(ang-pair["ang"]) < 1.0):
                hits.append(rec)
    return hits, near


def main():
    pairs = load_census_pairs()
    for pair in pairs:
        print(f"\n=== pair f({pair['face']},{pair['face']}) "
              f"h={pair['h']} ang={pair['ang']} mid={pair['mid']}")
        tgt = next(t for t in TARGETS if t["face"] == pair["face"])
        for fname in tgt["files"]:
            path = f"{EMDIR}/{fname}"
            verts, tris = load_em(path)
            hits, near = scan(verts, tris, pair)
            att = re.search(r"_a(\d+)\.txt", fname).group(1)
            pass_name = "first" if int(att) < 20 else "retry"
            if hits:
                h = hits[0]
                print(f"  {fname} ({pass_name}): EXACT MATCH "
                      f"e={h['edge']} tris={h['tris']} "
                      f"h=({h['h'][0]:.4f},{h['h'][1]:.4f}) "
                      f"ang={h['ang']:.2f} cls={h['cls']}")
                print(f"    verts6: edge={h['verts6'][0]}/{h['verts6'][1]} "
                      f"apex={h['verts6'][2]}/{h['verts6'][3]}")
            else:
                best = near[0] if near else None
                print(f"  {fname} ({pass_name}): no match "
                      f"({len(near)} near>170 pairs)"
                      + (f" best h=({best['h'][0]:.4f},{best['h'][1]:.4f})"
                         f" ang={best['ang']:.2f}"
                         f" cls={best['cls']} e={best['edge']}"
                         if best else ""))
        # also: exact-index check (are census tri vertex POSITIONS present
        # in the emission at all?)
        print("  -- census pair vertex positions in emissions:")
        for fname in tgt["files"]:
            path = f"{EMDIR}/{fname}"
            verts, tris = load_em(path)
            # census final positions of the 6 verts come from p0
            # postweldwind obj; find nearest emission vertex per apex
            # via pair mid geometry: skip (positions already checked via
            # scan radius)


if __name__ == "__main__":
    main()
