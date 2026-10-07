#!/usr/bin/env python3
"""s81: fresh census of the REAL fold debt after SEAM-GLUE DEVIATION
GUARD (drill REAL 856) — verify class B is dead, rank class C
(fillet-fillet / CDT messes) and the new tails per face-pair.
"""
import re
from collections import Counter

SRC = "forensics/s81_drill_full.txt"

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

    # class B verification: Plane x Cylinder should be near-dead
    print("\n== class B check: Plane x Cylinder (REAL) ==")
    b = [p for p in pairs if set(p["types"]) == {"Plane", "Cylinder"}]
    print(f"  total: {len(b)}")
    for fp, n in Counter(p["faces"] for p in b).most_common(8):
        ex = next(p for p in b if p["faces"] == fp)
        print(f"  ({fp[0]},{fp[1]}): {n:3d}  [{ex['name']}]"
              f"  h=({ex['h'][0]:.4f},{ex['h'][1]:.4f})  ang={ex['ang']:.1f}")

    # class C: Nurbs-involved
    print("\n== class C: Nurbs-involved (REAL) ==")
    c = [p for p in pairs if any("Nurbs" in t for t in p["types"])]
    print(f"  total: {len(c)}")
    for fp, n in Counter(p["faces"] for p in c).most_common(15):
        ex = next(p for p in c if p["faces"] == fp)
        print(f"  ({fp[0]},{fp[1]}): {n:3d}  [{ex['name']}] {'x'.join(ex['types'])}"
              f"  h=({ex['h'][0]:.4f},{ex['h'][1]:.4f})  ang={ex['ang']:.1f}")

    # top face-pair families overall
    print("\n== top 25 face-pair families (REAL) ==")
    for fp, n in Counter(p["faces"] for p in pairs).most_common(25):
        ex = next(p for p in pairs if p["faces"] == fp)
        print(f"  ({fp[0]},{fp[1]}): {n:3d}  [{ex['name']}] {'x'.join(ex['types'])}"
              f"  h=({ex['h'][0]:.4f},{ex['h'][1]:.4f})  ang={ex['ang']:.1f}"
              f"  areas=({ex['areas'][0]:.4f},{ex['areas'][1]:.4f})")


if __name__ == "__main__":
    main()
