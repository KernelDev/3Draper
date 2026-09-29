#!/usr/bin/env python3
"""s65 diag 9: analyze the failing CDT case for f9 (lune).
Load the .cdtfail dump, render both triangulations (legacy vs CDT) as
SVG to see the crescent hole and the CDT's failure mode.
"""
import sys
import glob
from collections import defaultdict

PATH = sorted(glob.glob("/tmp/cdtfail/brep1086_f9_Cone.cdtfail.txt"))[0]

def main():
    mode = None
    header = {}
    boundary, interior, holes = [], [], []
    legacy, cdt = [], []
    hole = None
    for line in open(PATH):
        line = line.strip()
        if not line:
            continue
        if line in ("BOUNDARY", "HOLES", "INTERIOR", "LEGACY", "CDT"):
            mode = line
            continue
        if mode is None:
            for kv in line.split():
                k, v = kv.split("=", 1)
                header[k] = int(v)
            continue
        parts = line.split()
        if mode == "BOUNDARY":
            boundary.append((float(parts[0]), float(parts[1])))
        elif mode == "HOLES":
            if parts[0] == "HOLE":
                hole = []
                holes.append(hole)
            else:
                hole.append((float(parts[0]), float(parts[1])))
        elif mode == "INTERIOR":
            interior.append((float(parts[0]), float(parts[1])))
        elif mode == "LEGACY":
            legacy.append(tuple(int(x) for x in parts))
        elif mode == "CDT":
            cdt.append(tuple(int(x) for x in parts))

    print(header)
    print(f"boundary={len(boundary)} holes={[len(h) for h in holes]} interior={len(interior)}")
    print(f"legacy={len(legacy)} cdt={len(cdt)}")

    n_b = len(boundary)
    def rim_edges():
        rims = set()
        for i in range(n_b):
            j = (i + 1) % n_b
            rims.add((min(i, j), max(i, j)))
        return rims

    def analyze(name, tris, npoints):
        rims = rim_edges()
        ec = defaultdict(int)
        for t in tris:
            for k in range(3):
                a, b = t[k], t[(k + 1) % 3]
                if a != b:
                    ec[(min(a, b), max(a, b))] += 1
        bnd = [e for e, n in ec.items() if n == 1]
        rim_bnd = [e for e in bnd if e in rims]
        extra = [e for e in bnd if e not in rims]
        missing_rim = [e for e in rims if e not in ec]
        print(f"\n{name}: tris={len(tris)} bnd={len(bnd)} rim_bnd={len(rim_bnd)} "
              f"extra_bnd={len(extra)} missing_rim={len(missing_rim)}")
        if extra:
            print(f"  extra edges (first 25): {extra[:25]}")
        if missing_rim:
            print(f"  missing rim edges: {missing_rim}")
        return extra, missing_rim

    analyze("LEGACY", legacy, len(boundary))
    analyze("CDT", cdt, len(boundary))

    # render SVG
    allpts = boundary + interior
    xs = [p[0] for p in allpts]; ys = [p[1] for p in allpts]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    W, H = 900, 400
    pad = 40
    def tx(x): return pad + (x - x0) / (x1 - x0) * (W - 2 * pad)
    def ty(y): return H - pad - (y - y0) / (y1 - y0) * (H - 2 * pad)

    for name, tris in (("legacy", legacy), ("cdt", cdt)):
        svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}">']
        svg.append(f'<rect width="{W}" height="{H}" fill="white"/>')
        for t in tris:
            pts = " ".join(f"{tx(allpts[i][0]):.1f},{ty(allpts[i][1]):.1f}" for i in t if i < len(allpts))
            svg.append(f'<polygon points="{pts}" fill="none" stroke="#888" stroke-width="0.5"/>')
        # boundary polygon
        bp = " ".join(f"{tx(p[0]):.1f},{ty(p[1]):.1f}" for p in boundary)
        svg.append(f'<polygon points="{bp}" fill="none" stroke="blue" stroke-width="1.5"/>')
        for i, p in enumerate(interior):
            svg.append(f'<circle cx="{tx(p[0]):.1f}" cy="{ty(p[1]):.1f}" r="3" fill="red"/>')
        svg.append("</svg>")
        open(f"/tmp/s65_{name}_f9.svg", "w").write("\n".join(svg))
    print("\nSVG: /tmp/s65_legacy_f9.svg /tmp/s65_cdt_f9.svg")

if __name__ == "__main__":
    main()
