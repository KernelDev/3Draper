#!/usr/bin/env python3
"""s79: deep dive into top REAL pair families in HM/HOUSING —
geometry (h, areas, midpoint), face types, self vs cross.
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
RE_MID = re.compile(r"mid=\(([^)]+)\)")
RE_TRIS = re.compile(r"tris=\((\[[^]]*\]),(\[[^]]*\])\)")
RE_SN = re.compile(r"snAng=(\S+)")


def parse_line(line):
    m = RE_HEAD.match(line)
    if not m:
        return None
    tags = m.group("tags")
    if "SUBTOL" in tags or "EXEMPT" in tags:
        return None
    mf = RE_FACES.search(line)
    mt = RE_TYPES.search(line)
    ma = RE_AREAS.search(line)
    mh = RE_H.search(line)
    mr = RE_ANG.search(line)
    mm = RE_MID.search(line)
    mtr = RE_TRIS.search(line)
    if not (mf and mt and ma and mh):
        return None
    return {
        "name": m.group("name"),
        "faces": (int(mf.group(1)), int(mf.group(2))),
        "types": mt.group(1),
        "areas": (float(ma.group(1)), float(ma.group(2))),
        "h": (float(mh.group(1)), float(mh.group(2))),
        "ang": float(mr.group(1)) if mr else 0.0,
        "mid": mm.group(1) if mm else "",
        "tris": (mtr.group(1), mtr.group(2)) if mtr else None,
    }


def main():
    pairs = []
    with open(SRC) as f:
        for line in f:
            p = parse_line(line)
            if p:
                pairs.append(p)

    for name, top_pairs in (("HOUSING_MIRROR",
                             [(57, 58), (26, 26), (3, 121), (147, 148),
                              (259, 260)]),
                            ("HOUSING",
                             [(49, 178), (227, 228), (144, 145),
                              (175, 176), (228, 228)])):
        print(f"############ {name} ############")
        for fp in top_pairs:
            sel = [p for p in pairs
                   if p["name"] == name and p["faces"] == fp]
            if not sel:
                continue
            hs = sorted(max(p["h"]) for p in sel)
            ar = sorted(max(p["areas"]) for p in sel)
            print(f"\n== faces={fp} n={len(sel)} types={sel[0]['types']}")
            print(f"   h: min={hs[0]:.4f} med={hs[len(hs)//2]:.4f} "
                  f"max={hs[-1]:.4f}")
            print(f"   max-area: min={ar[0]:.5f} med={ar[len(ar)//2]:.5f} "
                  f"max={ar[-1]:.5f}")
            print(f"   mids: {[s['mid'] for s in sel[:4]]}")
            # angular distribution
            angs = sorted(p["ang"] for p in sel)
            print(f"   ang: min={angs[0]:.1f} med={angs[len(angs)//2]:.1f} "
                  f"max={angs[-1]:.1f}")
            # triangle index spread: same tri neighborhood or spread?
            if sel[0]["tris"]:
                t0 = sel[0]["tris"][0]
                print(f"   sample tris: {sel[0]['tris']}")
        print()


if __name__ == "__main__":
    main()
