#!/usr/bin/env python3
"""Session-56 decisive test: does the P2 complement polygon OVERLAP the
primary triangulation region on the f198 family (dirty) vs f226 (clean)?

Reads TRI_INPUT dumps (boundary + holes + interior chain + primary tris)
and the p2_uv_area from the complement log, then checks the area balance:
    primary_uv_area + p2_uv_area  vs  domain_uv_area
Overshoot => the P2 polygon is NOT the complement of the primary region.
"""
import math
import re
import sys
from pathlib import Path

tri_dir = Path(sys.argv[1])
log_path = Path(sys.argv[2])


def parse(path):
    b, holes, interior, tris, mode = [], [], [], [], None
    cur_hole = None
    for line in path.read_text().splitlines():
        if line.startswith("type="):
            continue
        if line == "boundary":
            mode = "b"; continue
        if line == "interior":
            mode = "i"; continue
        if line == "tris":
            mode = "t"; continue
        if line.startswith("hole "):
            cur_hole = []
            holes.append(cur_hole)
            mode = "h"; continue
        if line.startswith("b ") and mode == "b":
            _, u, v = line.split(); b.append((float(u), float(v)))
        elif line.startswith("i ") and mode == "i":
            _, u, v = line.split(); interior.append((float(u), float(v)))
        elif line.startswith("h ") and mode == "h":
            _, u, v = line.split(); cur_hole.append((float(u), float(v)))
        elif line.startswith("t ") and mode == "t":
            _, a, c, d = line.split(); tris.append((int(a), int(c), int(d)))
    # index space: boundary 0..n_b, then holes, then interior
    pts = list(b)
    for h in holes:
        pts.extend(h)
    pts.extend(interior)
    return b, holes, interior, tris, pts


def shoelace(poly):
    s = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2


def tri_area(pts, t):
    (x1, y1), (x2, y2), (x3, y3) = pts[t[0]], pts[t[1]], pts[t[2]]
    return abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2


# p2 areas from the complement log (label like fbrep62542_f198_Torus)
p2_uv = {}
pat = re.compile(r"\[(\w+?)_Torus\] spike-chain complement: added (\d+) triangles.*?p2_uv_area=([0-9.e+-]+)")
for line in log_path.read_text(errors="replace").splitlines():
    m = pat.search(line)
    if m:
        face = re.search(r"(f\d+)$", m.group(1))
        if face:
            # keep the LAST occurrence (final pass)
            p2_uv[face.group(1)] = (int(m.group(2)), float(m.group(3)))

print(f"{'face':<22}{'domain':>10}{'primary':>10}{'p2_uv':>10}{'prim+p2':>10}{'overshoot':>10}  verdict")
for path in sorted(tri_dir.glob("tri_*.txt")):
    b, holes, interior, tris, pts = parse(path)
    if not interior:
        continue
    header = [l for l in path.read_text().splitlines() if l.startswith("type=")][0]
    label = re.search(r"label=(\S+)", header)
    if not label:
        continue
    lab = label.group(1)
    short = re.search(r"(f\d+)_", lab)
    if not short or short.group(1) not in ("f198", "f199", "f200", "f226", "f227", "f38"):
        continue
    key = lab.split("_", 1)[1].split("_")[0] if False else short.group(1)
    domain = shoelace(b) - sum(shoelace(h) for h in holes)
    primary = sum(tri_area(pts, t) for t in tris)
    p2 = p2_uv.get(key, (None, None))[1]
    if p2 is None:
        continue
    total = primary + p2
    over = total - domain
    verdict = "OVERLAP!" if over > 0.01 * domain else "ok"
    print(f"{lab:<22}{domain:10.4f}{primary:10.4f}{p2:10.4f}{total:10.4f}{over:10.4f}  {verdict}")
