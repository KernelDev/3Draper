#!/usr/bin/env python3
"""s80: fresh census of the REAL (non-SUBTOL, non-EXEMPT) fold debt
after PLANAR_FAN_GUARD (drill REAL 922).

Focus: separate class B (tangential slivers on CLEAN Plane meshes,
Plane x Cyl G1-joint, h~0.05 > eff_tol 0.03) from class C
(fillet-fillet / CDT messes, Nurbs side) and the residual tails,
per face-pair, with h/area/ang detail for the top families.
"""
import re
from collections import Counter, defaultdict

SRC = "forensics/s80_drill_full.txt"

RE_HEAD = re.compile(
    r"^\[(?P<tags>[^\]]*)\]\s+brep_idx=(?P<brep>\d+)\s+(?P<name>\S+)")
RE_FACES = re.compile(r"faces=\((\d+),(\d+)\)")
RE_TYPES = re.compile(r"types=\((.*?)\) step=")
RE_AREAS = re.compile(r"areas=\(([^,]+),([^)]+)\) h=")
RE_H = re.compile(r"\bh=\(([^,]+),([^)]+)\)")
RE_ANG = re.compile(r"ang=([\d.]+)")


def parse_line(line):
    m = RE_HEAD.match(line)
    if not m:
        return None
    tags = m.group("tags")
    if "SUBTOL" in tags or "EXEMPT" in tags:
        return None  # REAL only
    cls = re.sub(r"\s*(SUBTOL|TANGENT-EXEMPT|EXEMPT)\s*", "", tags).strip()
    mf = RE_FACES.search(line)
    mt = RE_TYPES.search(line)
    ma = RE_AREAS.search(line)
    mh = RE_H.search(line)
    mr = RE_ANG.search(line)
    if not (mf and mt and ma and mh):
        return None
    types = [t.strip() for t in mt.group(1).split(",")]
    types = [re.sub(r"\(.*", "", t) for t in types]
    return {
        "brep": int(m.group("brep")),
        "name": m.group("name"),
        "cls": cls,
        "faces": (int(mf.group(1)), int(mf.group(2))),
        "types": tuple(sorted(types[:2])),
        "areas": (float(ma.group(1)), float(ma.group(2))),
        "h": (float(mh.group(1)), float(mh.group(2))),
        "ang": float(mr.group(1)) if mr else 0.0,
    }


def main():
    pairs = []
    with open(SRC) as f:
        for line in f:
            p = parse_line(line)
            if p:
                pairs.append(p)

    print(f"REAL pairs total: {len(pairs)}")
    by_brep = Counter(p["name"] for p in pairs)
    for name, n in by_brep.most_common():
        print(f"  {name}: {n}")

    print("\n== by type-pair ==")
    for tp, n in Counter(p["types"] for p in pairs).most_common(15):
        print(f"  {n:5d}  {tp}")

    # Class B: Plane|Cylinder pairs (tangential G1 joints, clean Plane)
    print("\n== class B candidates: Plane x Cylinder (REAL) ==")
    b = [p for p in pairs if set(p["types"]) == {"Plane", "Cylinder"}]
    print(f"  total: {len(b)}")
    by_face = Counter(p["faces"] for p in b)
    for fp, n in by_face.most_common(12):
        ex = next(p for p in b if p["faces"] == fp)
        print(f"  ({fp[0]},{fp[1]}): {n:3d}  h=({ex['h'][0]:.4f},{ex['h'][1]:.4f})"
              f"  areas=({ex['areas'][0]:.4f},{ex['areas'][1]:.4f})  ang={ex['ang']:.1f}")

    # Class C: Nurbs-involved pairs
    print("\n== class C candidates: Nurbs-involved (REAL) ==")
    c = [p for p in pairs if any("Nurbs" in t for t in p["types"])]
    print(f"  total: {len(c)}")
    by_face = Counter(p["faces"] for p in c)
    for fp, n in by_face.most_common(15):
        ex = next(p for p in c if p["faces"] == fp)
        print(f"  ({fp[0]},{fp[1]}): {n:3d}  types={'x'.join(ex['types'])}"
              f"  h=({ex['h'][0]:.4f},{ex['h'][1]:.4f})  ang={ex['ang']:.1f}")

    # Top face-pair families overall
    print("\n== top 20 face-pair families (REAL) ==")
    for fp, n in Counter(p["faces"] for p in pairs).most_common(20):
        ex = next(p for p in pairs if p["faces"] == fp)
        print(f"  ({fp[0]},{fp[1]}): {n:3d}  [{ex['name']}] {'x'.join(ex['types'])}"
              f"  h=({ex['h'][0]:.4f},{ex['h'][1]:.4f})  ang={ex['ang']:.1f}")


if __name__ == "__main__":
    main()
