#!/usr/bin/env python3
"""s91: diff fold-pair census between two fold_face_probe dumps (audit ON vs OFF).

Pair identity across runs: (brep, frozenset of the two triangles' SORTED
vertex triples) — invariant under the audit's 1,2-index swap, immune to
Nurbs type commas. Reports pairs created / repaired / reclassified by the
post-weld winding audit + a per-BREP REAL delta table.
"""
import re
import sys
from collections import defaultdict

PAIR_RE = re.compile(
    r"^\[(?P<class>FOLD-OVER|DOUBLE-BROKEN|CURVED-180|WINDING-FLIP)(?P<sliver>[A-Z+]+)?"
    r"(?P<subtol> SUBTOL)?\]"
    r"(?P<exempt>\S*?) brep_idx=(?P<brep>\d+) (?P<name>\S+) \(BREP#(?P<brepid>\d+)\) BREP#\d+ "
    r"ang=(?P<ang>[\d.]+) faces=\((?P<f0>\d+),(?P<f1>\d+)\) "
    r"types=\((?P<t0>.*?),(?P<t1>.*?)\) "
    r"step=\((?P<s0>[^,]*),(?P<s1>[^)]*?)\) "
    r"tris=\(\[(?P<a0>\d+), (?P<a1>\d+), (?P<a2>\d+)\],"
    r"\[(?P<b0>\d+), (?P<b1>\d+), (?P<b2>\d+)\]\) "
    r"areas=\((?P<ar0>[\d.]+),(?P<ar1>[\d.]+)\) h=\((?P<h0>[\d.]+),(?P<h1>[\d.]+)\)"
)


def load(path):
    pairs = {}
    n_lines = 0
    n_parsed = 0
    with open(path) as fh:
        for line in fh:
            line = line.rstrip("\n")
            m = PAIR_RE.match(line)
            if not m:
                continue
            n_parsed += 1
            t0v = tuple(sorted((int(m["a0"]), int(m["a1"]), int(m["a2"]))))
            t1v = tuple(sorted((int(m["b0"]), int(m["b1"]), int(m["b2"]))))
            key = (m["brep"], frozenset((t0v, t1v)))
            real_class = m["class"] in ("FOLD-OVER", "DOUBLE-BROKEN")
            is_subtol = " SUBTOL" in (m["subtol"] or "")
            pairs[key] = {
                "name": m["name"],
                "brep": m["brep"],
                "class": m["class"],
                "sliver": m["sliver"] or "",
                "subtol": is_subtol,
                "real": real_class and not is_subtol,
                "faces": (m["f0"], m["f1"]),
                "types": (m["t0"], m["t1"]),
                "h": (float(m["h0"]), float(m["h1"])),
                "areas": (float(m["ar0"]), float(m["ar1"])),
                "ang": float(m["ang"]),
                "verts": (m["a0"] + " " + m["a1"] + " " + m["a2"],
                          m["b0"] + " " + m["b1"] + " " + m["b2"]),
            }
    return pairs


def main(on_path, off_path, show_limit=60):
    on = load(on_path)
    off = load(off_path)
    on_keys, off_keys = set(on), set(off)

    new = sorted(on_keys - off_keys, key=lambda k: (int(k[0]), min(k[1])))
    gone = sorted(off_keys - on_keys, key=lambda k: (int(k[0]), min(k[1])))
    reclassed = sorted(
        [k for k in (on_keys & off_keys)
         if (on[k]["class"], on[k]["subtol"]) != (off[k]["class"], off[k]["subtol"])],
        key=lambda k: (int(k[0]), min(k[1])))

    print(f"pairs ON={len(on)} OFF={len(off)}")
    print(f"NEW={len(new)}  GONE={len(gone)}  RECLASSIFIED={len(reclassed)}")

    def realcount(d, keys):
        return sum(1 for k in keys if d[k]["real"])

    print(f"REAL ON={realcount(on, on_keys)} OFF={realcount(off, off_keys)} "
          f"delta={realcount(on, on_keys) - realcount(off, off_keys):+d}")

    print("\n=== REAL delta per BREP (name) ===")
    breps = sorted({int(k[0]) for k in on_keys | off_keys})
    for b in breps:
        on_b = {k for k in on_keys if int(k[0]) == b}
        off_b = {k for k in off_keys if int(k[0]) == b}
        d = realcount(on, on_b) - realcount(off, off_b)
        if d != 0 or len(on_b) != len(off_b):
            nm = ((next(iter(on_b), None) and on.get(next(iter(on_b))))
                  or (next(iter(off_b), None) and off.get(next(iter(off_b)))))
            nm = nm["name"] if nm else "?"
            print(f"  brep {b} {nm}: real {realcount(off, off_b)} -> {realcount(on, on_b)} "
                  f"({d:+d}), census {len(off_b)} -> {len(on_b)}")

    print("\n=== NEW pairs that count as REAL (the regression core) ===")
    nshown = 0
    for k in new:
        p = on[k]
        if p["real"]:
            print(f"  brep={k[0]} {p['name']} faces={p['faces']} types={p['types']} "
                  f"h={p['h']} areas={p['areas']} ang={p['ang']} -> {p['class']}"
                  f"{'/SUBTOL' if p['subtol'] else ''}")
            nshown += 1
            if nshown >= show_limit:
                break
    print(f"  (total NEW-real: {sum(1 for k in new if on[k]['real'])})")

    print("\n=== NEW pairs census by class ===")
    cnt = defaultdict(int)
    for k in new:
        p = on[k]
        cnt[f"{p['class']}{'/SUBTOL' if p['subtol'] else ''}"] += 1
    for r, n in sorted(cnt.items(), key=lambda x: -x[1]):
        print(f"  {r}: {n}")

    print("\n=== GONE pairs census by class (was) ===")
    cnt2 = defaultdict(int)
    for k in gone:
        p = off[k]
        cnt2[f"{p['class']}{'/SUBTOL' if p['subtol'] else ''}"] += 1
    for r, n in sorted(cnt2.items(), key=lambda x: -x[1]):
        print(f"  {r}: {n}")

    print("\n=== RECLASSIFIED pairs (OFF -> ON) ===")
    cnt3 = defaultdict(int)
    for k in reclassed:
        o, n = off[k], on[k]
        cnt3[f"{o['class']}{'/S' if o['subtol'] else ''}->{n['class']}{'/S' if n['subtol'] else ''}"] += 1
    for r, n in sorted(cnt3.items(), key=lambda x: -x[1]):
        print(f"  {r}: {n}")

    print("\n=== GONE pairs detail (audit-repaired WINDING-FLIPs — intended wins) ===")
    nshown = 0
    for k in gone:
        p = off[k]
        if p["class"] == "WINDING-FLIP" and not p["subtol"]:
            print(f"  brep={k[0]} {p['name']} faces={p['faces']} h={p['h']}")
            nshown += 1
            if nshown >= 20:
                break


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
