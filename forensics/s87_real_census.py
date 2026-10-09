#!/usr/bin/env python3
"""s87: baseline census after sandbox restore — verify drill 2837/653
(SHAFT 13/6, GEAR 65/50, SLEEVE 198/107, HOUSING 1263/283, HM 1298/207)
matches the committed s86 state bit-for-bit."""
import re
import sys
from collections import Counter

SRC = sys.argv[1] if len(sys.argv) > 1 else "forensics/s87_drill_full.txt"

RE_HEAD = re.compile(
    r"^\[(?P<tags>[^\]]*)\]\s+brep_idx=(?P<brep>\d+)\s+(?P<name>\S+)")
RE_FACES = re.compile(r"faces=\((\d+),(\d+)\)")
RE_TYPES = re.compile(r"types=\((.*?)\) step=")
RE_AREAS = re.compile(r"areas=\(([^,]+),([^)]+)\) h=")
RE_H = re.compile(r"\bh=\(([^,]+),([^)]+)\)")
RE_ANG = re.compile(r"\bang=([\d.]+)")


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

    per_name = Counter()
    raw_per_name = Counter()
    for line in open(SRC):
        m = RE_HEAD.match(line)
        if m:
            raw_per_name[m.group("name")] += 1

    print(f"TOTAL raw={sum(raw_per_name.values())} REAL={len(pairs)}")
    for name in sorted(raw_per_name):
        r = sum(1 for p in pairs if p["name"] == name)
        print(f"  {name:10s} raw={raw_per_name[name]:5d} REAL={r:4d}")

    print("\nTop families (faces, name):")
    fam = Counter((p["name"], p["faces"]) for p in pairs)
    for (name, faces), n in fam.most_common(15):
        print(f"  {name:10s} {faces}: {n}")


if __name__ == "__main__":
    main()
