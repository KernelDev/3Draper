#!/usr/bin/env python3
"""s79: fresh census of the REAL (non-SUBTOL, non-EXEMPT) fold debt
in HM 407 / HOUSING 366 after the s78 SAIL+TSZ baseline.

Breaks REAL pairs down by: type-pair class, face pair, h-band, area
class — to locate the lip-wedge class + tails for the s79 attack.
"""
import re
from collections import Counter, defaultdict

SRC = "forensics/s79_drill_full.txt"

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
    print("by BREP:", dict(by_brep))
    print()

    for target in ("HOUSING_MIRROR", "HOUSING"):
        sel = [p for p in pairs if p["name"] == target]
        if not sel:
            continue
        print(f"=== {target}: {len(sel)} REAL ===")
        # by type-pair class
        by_types = Counter(p["types"] for p in sel)
        print("by surface-type pair:")
        for t, c in by_types.most_common(12):
            print(f"  {t}: {c}")
        # by h band (max apex height)
        print("by max-h band:")
        bands = [(0, 0.05), (0.05, 0.1), (0.1, 0.3), (0.3, 1.0),
                 (1.0, 3.0), (3.0, 1e9)]
        for lo, hi in bands:
            c = sum(1 for p in sel if lo <= max(p["h"]) < hi)
            print(f"  h in [{lo},{hi}): {c}")
        # top face pairs
        by_faces = Counter(p["faces"] for p in sel)
        print("top face pairs:")
        for f_, c in by_faces.most_common(15):
            ex = next(p for p in sel if p["faces"] == f_)
            print(f"  faces={f_} {ex['types']}: {c}")
        # same-face vs cross-face
        same = sum(1 for p in sel if p["faces"][0] == p["faces"][1])
        print(f"same-face (self) pairs: {same}, cross-face: "
              f"{len(sel) - same}")
        # class breakdown
        by_cls = Counter(p["cls"] for p in sel)
        print("by class:", dict(by_cls))
        print()


if __name__ == "__main__":
    main()
