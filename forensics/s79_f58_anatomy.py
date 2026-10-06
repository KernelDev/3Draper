#!/usr/bin/env python3
"""s79: 3D anatomy of the HM (57,58) fold cluster + f26 self-folds.

Uses the FINAL_OBJS merged mesh (brep4 HM): obj + fmap + facemap.
Extracts the fold-pair triangles of faces 57/58 and 26 from the
probe's pair lines, then analyzes their 3D structure: band rows,
levels, what the ladder looks like, where the 180-deg folds sit.
"""
import re
from collections import defaultdict

OBJ = ("/tmp/s79_final/brep4_HOUSING_MIRROR (BREP#62542).obj")
FMAP = ("/tmp/s79_final/brep4_HOUSING_MIRROR (BREP#62542).fmap")
FACEMAP = ("/tmp/s79_final/brep4_HOUSING_MIRROR (BREP#62542).facemap")
PAIRS = "forensics/s79_drill_full.txt"


def load():
    verts = []
    tris = []
    with open(OBJ) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                a, b, c = [int(x) for x in line.split()[1:4]]
                tris.append((a - 1, b - 1, c - 1))
    t2f = {}
    with open(FMAP) as f:
        for line in f:
            if line.startswith("t "):
                _, ti, fid = line.split()
                t2f[int(ti)] = int(fid)
    f2t = defaultdict(list)
    for ti, fid in t2f.items():
        f2t[fid].append(ti)
    return verts, tris, t2f, f2t


def main():
    verts, tris, t2f, f2t = load()
    print(f"verts={len(verts)} tris={len(tris)}")

    # face types
    ftype = {}
    with open(FACEMAP) as f:
        for line in f:
            if line.startswith("f "):
                parts = line.split()
                fid = int(parts[1])
                st = parts[2]
                fwd = parts[4] if len(parts) > 4 else "true"
                ftype[fid] = f"{st}{'!fwd' if fwd == 'false' else ''}"

    # collect the (57,58) + (26,26) fold pair triangles from probe lines
    RE = re.compile(
        r"^\[FOLD[^\]]*\] brep_idx=4 \S+.*faces=\((\d+),(\d+)\).*"
        r"tris=\((\[[^]]*\]),(\[[^]]*\])\).*h=\(([^,]+),([^)]+)\)")
    pairs_5758 = []
    pairs_2626 = []
    with open(PAIRS) as f:
        for line in f:
            if "SUBTOL" in line or "EXEMPT" in line:
                continue
            m = RE.match(line)
            if not m:
                continue
            fa, fb = int(m.group(1)), int(m.group(2))
            ta = [int(x) for x in m.group(3)[1:-1].split(",")]
            tb = [int(x) for x in m.group(4)[1:-1].split(",")]
            h = (float(m.group(5)), float(m.group(6)))
            if (fa, fb) == (57, 58):
                pairs_5758.append((ta, tb, h))
            elif (fa, fb) == (26, 26):
                pairs_2626.append((ta, tb, h))

    print(f"(57,58) pairs: {len(pairs_5758)}, (26,26) self: {len(pairs_2626)}")

    # ── geometry of f58 (the LUNE ladder) �─
    f58_tris = f2t.get(58, [])
    f57_tris = f2t.get(57, [])
    f26_tris = f2t.get(26, [])
    print(f"f57 tris: {len(f57_tris)}, f58 tris: {len(f58_tris)}, "
          f"f26 tris: {len(f26_tris)}")

    def tri_centroid(t):
        return tuple(
            (verts[t[0]][k] + verts[t[1]][k] + verts[t[2]][k]) / 3
            for k in range(3))

    # bbox of each face's triangles
    for fid, tl in ((57, f57_tris), (58, f58_tris), (26, f26_tris)):
        if not tl:
            continue
        cs = [tri_centroid(tris[t]) for t in tl]
        mins = [min(c[k] for c in cs) for k in range(3)]
        maxs = [max(c[k] for c in cs) for k in range(3)]
        print(f"f{fid} ({ftype.get(fid)}): {len(tl)} tris, "
              f"bbox min=({mins[0]:.2f},{mins[1]:.2f},{mins[2]:.2f}) "
              f"max=({maxs[0]:.2f},{maxs[1]:.2f},{maxs[2]:.2f})")

    # ── the fold triangles themselves ──
    print("\n=== (57,58) fold pairs: h distribution + geometry ===")
    big = [p for p in pairs_5758 if max(p[2]) > 1.0]
    small = [p for p in pairs_5758 if max(p[2]) <= 1.0]
    print(f"h>1.0: {len(big)}, h<=1.0: {len(small)}")
    for ta, tb, h in big[:6]:
        ca, cb = tri_centroid(ta), tri_centroid(tb)
        # triangle areas
        import math
        def area(t):
            a, b, c = verts[t[0]], verts[t[1]], verts[t[2]]
            u = [b[k] - a[k] for k in range(3)]
            v = [c[k] - a[k] for k in range(3)]
            n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
            return math.sqrt(sum(x*x for x in n)) / 2
        # shared vertices?
        shared = set(ta) & set(tb)
        print(f"  h=({h[0]:.3f},{h[1]:.3f}) shared={len(shared)} "
              f"areas=({area(ta):.5f},{area(tb):.5f}) "
              f"ca=({ca[0]:.2f},{ca[1]:.2f},{ca[2]:.2f}) "
              f"cb=({cb[0]:.2f},{cb[1]:.2f},{cb[2]:.2f})")

    # ── f58 ladder structure: vertex z-histogram ──
    f58_v = set()
    for t in f58_tris:
        f58_v.update(tris[t])
    print(f"\nf58 distinct verts: {len(f58_v)}")
    zs = sorted(verts[v][2] for v in f58_v)
    print(f"f58 vert z: min={zs[0]:.3f} med={zs[len(zs)//2]:.3f} "
          f"max={zs[-1]:.3f}")
    # z-histogram in 0.5 bins
    hist = defaultdict(int)
    for z in zs:
        hist[round(z * 2) / 2] += 1
    print("f58 z-histogram (0.5 bins):",
          dict(sorted(hist.items())))

    # ── (26,26) self pairs geometry ──
    print("\n=== (26,26) self pairs ===")
    for ta, tb, h in pairs_2626[:5]:
        ca, cb = tri_centroid(ta), tri_centroid(tb)
        shared = set(ta) & set(tb)
        print(f"  h=({h[0]:.3f},{h[1]:.3f}) shared={len(shared)} "
              f"ca=({ca[0]:.2f},{ca[1]:.2f},{ca[2]:.2f})")


if __name__ == "__main__":
    main()
