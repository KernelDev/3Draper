#!/usr/bin/env python3
"""s63 diag: exact-match structure of pre-merge rims (local frame).

f16 (plane #1737) perimeter = BSPLINE#5577 (to cone f9) + LINE#5589
(to plane f20) + BSPLINE#5585 (to cone f15) + LINE#5590 (to plane f19).
Load all six neighbors' rims, exact-match f16's rim points (d<1e-9),
and characterize the unmatched ones geometrically (arc fit + position
along perimeter).
"""
import math
import os
from collections import defaultdict

DIR = "/home/z/my-project/scripts/fobjs1086"

def load_rim(fname):
    vs, tris = [], []
    for line in open(os.path.join(DIR, fname)):
        if line.startswith("v "):
            _, x, y, z = line.split()
            vs.append((float(x), float(y), float(z)))
        elif line.startswith("f "):
            a, b, c = line.split()[1:4]
            tris.append((int(a) - 1, int(b) - 1, int(c) - 1))
    e2t = defaultdict(list)
    for ti, (a, b, c) in enumerate(tris):
        for v0, v1 in ((a, b), (b, c), (c, a)):
            e2t[(min(v0, v1), max(v0, v1))].append(ti)
    rim = sorted({v for e in e2t if len(e2t[e]) == 1 for v in e})
    return [vs[v] for v in rim]

def main():
    f16 = load_rim("brep1086_f16_s1737_Plane.obj")
    others = {}
    for f, name in [(9, "brep1086_f9_s1730_Cone.obj"),
                    (15, "brep1086_f15_s1736_Cone.obj"),
                    (19, "brep1086_f19_s1740_Plane.obj"),
                    (20, "brep1086_f20_s1741_Plane.obj")]:
        others[f] = load_rim(name)

    # exact matches for each f16 rim point
    matched_with = {}
    for i, p in enumerate(f16):
        for f, rim in others.items():
            if any(math.dist(p, q) < 1e-9 for q in rim):
                matched_with[i] = f
                break
    unmatched = [i for i in range(len(f16)) if i not in matched_with]
    print(f"f16 rim: {len(f16)} pts; matched={len(matched_with)} unmatched={len(unmatched)}")
    from collections import Counter
    print("matched partner histogram:", Counter(matched_with.values()))

    # geometry of unmatched: cluster into runs by sequential proximity
    pts = [f16[i] for i in unmatched]
    # order along original rim: sort by index; consecutive indices = neighbors on rim
    runs = []
    cur = [unmatched[0]]
    for i in unmatched[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            runs.append(cur)
            cur = [i]
    runs.append(cur)
    print(f"unmatched runs (by rim index): {[len(r) for r in runs]}")
    for r in runs:
        if len(r) < 3:
            print(f"  run idx[{r[0]}..{r[-1]}]: {len(r)} pts (short)")
            continue
        seg = [f16[i] for i in r]
        # local circle fit of the polyline: use three-point circle through
        # first/mid/last to estimate curvature radius
        a, b, c = seg[0], seg[len(seg) // 2], seg[-1]
        ax, ay, az = a; bx, by, bz = b; cx, cy, cz = c
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-12:
            r_est = float("inf")
            center = None
        else:
            ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
            uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
            center = (ux, uy)
            r_est = math.hypot(ax - ux, ay - uy)
        span = sum(math.dist(seg[i], seg[i + 1]) for i in range(len(seg) - 1))
        print(f"  run idx[{r[0]}..{r[-1]}]: {len(r)} pts, arc_len={span:.4f}, chord={math.dist(seg[0], seg[-1]):.4f}, "
              f"3pt-circle R(xy)~{r_est:.4f}" + (f" c=({center[0]:.3f},{center[1]:.3f})" if center else ""))

    # also print rim-index layout of ALL points with match status
    layout = "".join(str(matched_with.get(i, ".")) if matched_with.get(i, ".") != "." else "." for i in range(len(f16)))
    print("\nrim layout (digit=partner fid, .=unmatched):")
    for s in range(0, len(layout), 60):
        idx = f"[{s:3d}]"
        print(f"  {idx} {layout[s:s+60]}")

if __name__ == "__main__":
    main()
