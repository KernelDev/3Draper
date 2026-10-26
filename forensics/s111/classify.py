#!/usr/bin/env python3
"""s111: classify the 113 REAL >170deg pairs (Vulcan 56 + Spit-Fire 57).

Parses fold_face_probe dumps, keeps only REAL pair lines (no SUBTOL marker),
joins full-precision eff_tol (s109_eff_tol_probe), and builds the
classification tables: class x shape x same/cross-face x types x h/eff_tol.
"""
import re
import sys
from collections import defaultdict

VULC = "forensics/s111/vulcan_census.txt"
SPIT = "forensics/s111/spitfire_census.txt"

# full-precision eff_tol per BREP# (s109_eff_tol_probe, this session)
EFF_TOL = {
    # Vulcan
    10203: 0.015757570806264137,
    10204: 0.014186320015408817,
    10212: 0.01610411830935676,
    10217: 0.011154187100963818,
    10220: 0.01999774090415103,
    # Spit-Fire
    27977: 0.016687460013269498,
    31356: 0.019423024347968512,
    31357: 0.019217633779584343,
    31359: 0.015621669280118322,
    31360: 0.017858547324765217,
}

REAL_CLASSES = {"FOLD-OVERFAT", "FOLD-OVERSLIVER", "WINDING-FLIPFAT", "WINDING-FLIPSLIVER"}

LINE_RE = re.compile(
    r"^\[(?P<cls>[A-Z-]+)\] brep_idx=(?P<brep_idx>\d+) (?P<nauo>\S+) \(BREP#(?P<nauo_id>\d+)\) "
    r"BREP#(?P<brep>\d+) ang=(?P<ang>[\d.]+) faces=\((?P<f1>\d+),(?P<f2>\d+)\) "
    r"types=\((?P<types>.*)\) step=\(\d+,\d+\) "
    r"tris=\(\[(?P<t1v>\d+), (?P<t2v>\d+), (?P<t3v>\d+)\],\[(?P<u1v>\d+), (?P<u2v>\d+), (?P<u3v>\d+)\]\) "
    r"areas=\((?P<a1>[\d.]+),(?P<a2>[\d.]+)\) "
    r"h=\((?P<h1>[\d.]+),(?P<h2>[\d.]+)\)"
)


def tri_edges(t):
    return [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]


def shared_edge_dir(tri_a, tri_b):
    """Return 'opposite' (manifold-correct traversal), 'same' (inverted
    winding pair), or 'noshare' (no common edge)."""
    ea, eb = tri_edges(tri_a), tri_edges(tri_b)
    for (u, v) in ea:
        for (x, y) in eb:
            if u == x and v == y:
                return "same"        # both traverse u->v: inverted winding
            if u == y and v == x:
                return "opposite"    # correct manifold traversal
    return "noshare"


def split_top(s):
    """split 'A,B' on top-level comma (parens aware)"""
    parts, depth, cur = [], 0, ""
    for ch in s:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += ch
    parts.append(cur)
    return parts


def base_type(t):
    t = t.replace("!fwd", "").strip()
    m = re.match(r"([A-Za-z]+)", t)
    return m.group(1) if m else t


def parse(path):
    pairs = []
    for line in open(path):
        line = line.rstrip("\n")
        m = LINE_RE.match(line)
        if not m:
            continue
        cls = m.group("cls")
        if cls not in REAL_CLASSES:
            continue
        brep = int(m.group("brep"))
        h1, h2 = float(m.group("h1")), float(m.group("h2"))
        hmax = max(h1, h2)
        eff = EFF_TOL.get(brep)
        ratio = hmax / eff if eff else None
        tparts = split_top(m.group("types"))
        t1 = tparts[0] if len(tparts) >= 1 else "?"
        t2 = tparts[1] if len(tparts) >= 2 else "?"
        tri_a = (int(m.group("t1v")), int(m.group("t2v")), int(m.group("t3v")))
        tri_b = (int(m.group("u1v")), int(m.group("u2v")), int(m.group("u3v")))
        pairs.append({
            "edir": shared_edge_dir(tri_a, tri_b),
            "tri_a": tri_a, "tri_b": tri_b,
            "cls": cls,
            "brep": brep,
            "nauo": m.group("nauo"),
            "brep_idx": int(m.group("brep_idx")),
            "ang": float(m.group("ang")),
            "face1": int(m.group("f1")),
            "face2": int(m.group("f2")),
            "t1": t1,
            "t2": t2,
            "h1": h1, "h2": h2, "hmax": hmax,
            "eff": eff,
            "ratio": ratio,
            "coincident": "COINCIDENT" in line,
            "same_face": m.group("f1") == m.group("f2"),
            "raw": line,
        })
    return pairs


def fmt_ratio(r):
    return f"{r:.1f}x" if r is not None else "?"


def report(name, pairs):
    print(f"\n{'='*78}\n{name}: {len(pairs)} REAL pairs\n{'='*78}")

    # 1. class x shape
    by_cls = defaultdict(int)
    for p in pairs:
        c = p["cls"]
        mech = "FOLD-OVER" if c.startswith("FOLD") else "WINDING-FLIP"
        shape = "FAT" if c.endswith("FAT") else "SLIVER"
        by_cls[(mech, shape)] += 1
    print("\n[1] mechanism x shape:")
    for (mech, shape), n in sorted(by_cls.items()):
        print(f"    {mech:13s} x {shape:6s} : {n}")

    # 1b. verified mechanics: shared-edge traversal direction
    print("\n[1b] shared-edge traversal x census class (mechanism verification):")
    dir_cls = defaultdict(int)
    for p in pairs:
        dir_cls[(p["edir"], p["cls"].rsplit("FAT")[0].rsplit("SLIVER")[0])] += 1
    for (d, c), n in sorted(dir_cls.items()):
        print(f"    {d:9s} x {c:12s} : {n}")

    # 2. same-face vs cross-face
    same = sum(1 for p in pairs if p["same_face"])
    print(f"\n[2] same-face: {same} / cross-face: {len(pairs)-same}")

    # 3. per-BREP table
    print("\n[3] per-BREP breakdown (h/eff_tol = max margin of pairs):")
    per_brep = defaultdict(list)
    for p in pairs:
        per_brep[p["brep"]].append(p)
    for brep in sorted(per_brep):
        ps = per_brep[brep]
        ratios = [p["ratio"] for p in ps if p["ratio"]]
        same = sum(1 for p in ps if p["same_face"])
        cls_mix = defaultdict(int)
        face_pairs = defaultdict(int)
        for p in ps:
            mech = "FO" if p["cls"].startswith("FOLD") else "WF"
            shape = "FAT" if p["cls"].endswith("FAT") else "SLV"
            cls_mix[f"{mech}/{shape}"] += 1
            bt = (base_type(p["t1"]), base_type(p["t2"]))
            face_pairs[bt if bt[0] <= bt[1] else (bt[1], bt[0])] += 1
        mix = " ".join(f"{k}:{v}" for k, v in sorted(cls_mix.items()))
        ftpes = " ".join(f"{a}x{b}:{n}" for (a, b), n in sorted(face_pairs.items(), key=lambda x: -x[1]))
        faces = sorted({p["face1"] for p in ps} | {p["face2"] for p in ps})
        rmin = min(ratios) if ratios else 0
        rmax = max(ratios) if ratios else 0
        print(f"    BREP#{brep} {ps[0]['nauo']:12s} n={len(ps):2d} same={same:2d} "
              f"h/eT={rmin:.1f}-{rmax:.1f}x | {mix}")
        print(f"        faces involved: {faces}")
        print(f"        types: {ftpes}")

    # 4. h/eff_tol margin histogram
    print("\n[4] h/eff_tol margin histogram (max h of pair / full-precision eff_tol):")
    buckets = [(1, 2), (2, 4), (4, 8), (8, 16), (16, 100)]
    for lo, hi in buckets:
        n = sum(1 for p in pairs if p["ratio"] and lo <= p["ratio"] < hi)
        bar = "#" * n
        print(f"    {lo:3d}-{hi:<3d}x : {n:3d} {bar}")
    over = sum(1 for p in pairs if p["ratio"] and p["ratio"] >= 100)
    print(f"    >=100x   : {over:3d}")

    # 5. angle distribution
    a180 = sum(1 for p in pairs if p["ang"] >= 179.99)
    a175 = sum(1 for p in pairs if 175 <= p["ang"] < 179.99)
    a170 = sum(1 for p in pairs if p["ang"] < 175)
    print(f"\n[5] angle: >=179.99: {a180} | 175-179.99: {a175} | 170-175: {a170}")

    # 6. worst offenders by margin
    print("\n[6] top-8 worst by h/eff_tol margin:")
    for p in sorted(pairs, key=lambda x: -(x["ratio"] or 0))[:8]:
        sc = "same" if p["same_face"] else "cross"
        print(f"    {fmt_ratio(p['ratio']):>7s} BREP#{p['brep']} {p['nauo']:12s} "
              f"ang={p['ang']:6.2f} {sc} f=({p['face1']},{p['face2']}) "
              f"{base_type(p['t1'])}x{base_type(p['t2'])} h={p['hmax']:.4f} [{p['cls']}]")


def main():
    vulc = parse(VULC)
    spit = parse(SPIT)
    assert len(vulc) == 56, f"Vulcan REAL != 56: {len(vulc)}"
    assert len(spit) == 57, f"Spit-Fire REAL != 57: {len(spit)}"
    report("VULCAN 8500-02", vulc)
    report("SPIT-FIRE 8394-121", spit)
    # global cross-check vs brick_thin_round chronic profile
    print(f"\n{'='*78}\nGLOBAL: {len(vulc)+len(spit)} REAL pairs total\n{'='*78}")
    allp = vulc + spit
    coinc = sum(1 for p in allp if p["coincident"])
    print(f"COINCIDENT-flagged: {coinc}")
    same = sum(1 for p in allp if p["same_face"])
    print(f"same-face: {same} / cross-face: {len(allp)-same}")
    nurbs = sum(1 for p in allp if base_type(p["t1"]).startswith("Nurbs") or base_type(p["t2"]).startswith("Nurbs"))
    print(f"pairs involving a Nurbs face: {nurbs}")
    # instance dedup: same BREP# + class + faces + h = same defect in another NAUO instance
    seen, uniq = set(), []
    for p in allp:
        key = (p["brep"], p["cls"], p["face1"], p["face2"], round(p["h1"], 6), round(p["h2"], 6))
        if key not in seen:
            seen.add(key)
            uniq.append(p)
    VBREPS = {10203, 10204, 10212, 10217, 10220}
    vuniq = sum(1 for p in uniq if p["brep"] in VBREPS)
    print(f"instance-deduplicated unique defects: {len(uniq)} "
          f"(Vulcan {vuniq} + Spit-Fire {len(uniq)-vuniq}; "
          f"{len(allp)-len(uniq)} duplicate-instance copies)")


if __name__ == "__main__":
    main()
