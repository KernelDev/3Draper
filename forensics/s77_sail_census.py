#!/usr/bin/env python3
"""s77: census over DRAPPER_DUMP_TRI_INPUT dumps (drill brep4 HM + brep3
HOUSING): per face — interior total vs used in the FINAL triangles of
that call; flag the sail-class candidates (Nurbs, dropped >= 4 and
>= 25% of budget)."""
import glob
import re
import sys
from collections import defaultdict

def parse(path):
    bnd, inter, tris = [], [], []
    cur = None
    hdr = ""
    for line in open(path):
        if line.startswith("type="):
            hdr = line.strip()
            m = re.search(r"label=(\S+)", hdr)
            label = m.group(1) if m else "?"
        elif line.startswith("boundary"):
            cur = "b"
        elif line.startswith("interior"):
            cur = "i"
        elif line.startswith("tris"):
            cur = "t"
        else:
            p = line.split()
            if not p:
                continue
            if cur == "b" and p[0] == "b":
                bnd.append(1)
            elif cur == "i" and p[0] == "i":
                inter.append(len(inter))
            elif cur == "t" and p[0] == "t":
                tris.append((int(p[1]), int(p[2]), int(p[3])))
    return label, len(bnd), inter, tris

def main():
    base = sys.argv[1] if len(sys.argv) > 1 else "forensics/s77_dumps/tri_on"
    rows = []
    for path in sorted(glob.glob(f"{base}/*.txt")):
        label, nb, inter, tris = parse(path)
        if not inter:
            continue
        n_bah = nb  # no holes in these faces (sail class); approx
        used = set()
        for t in tris:
            used.update(t)
        n_int = len(inter)
        dropped = sum(1 for i in range(n_bah, n_bah + n_int) if i not in used)
        # fan degrees
        deg = defaultdict(int)
        for t in tris:
            for v in t:
                deg[v] += 1
        topdeg = max(deg.values()) if deg else 0
        if dropped > 0 or topdeg >= 15:
            rows.append((label, n_int, dropped, topdeg, len(tris), nb))
    rows.sort(key=lambda r: -r[2])
    print(f"{'label':44s} {'int':>4s} {'drop':>5s} {'topdeg':>6s} {'tris':>5s} {'bnd':>4s}")
    for label, n_int, dropped, topdeg, ntris, nb in rows:
        flag = "SAIL" if dropped >= 4 and dropped * 4 >= n_int else ""
        print(f"{label:44s} {n_int:4d} {dropped:5d} {topdeg:6d} {ntris:5d} {nb:4d} {flag}")

if __name__ == "__main__":
    main()
