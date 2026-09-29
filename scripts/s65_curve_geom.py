#!/usr/bin/env python3
"""s65 diag 4: curve geometry of the #1086 collar junction.

Dumps EDGE_CURVE geometry for: arc #5564, flap bsplines #5577-#5582,
f15 rim bsplines #5583-#5588, lines #5589-#5594 — endpoints, control
points, sampled polyline (via de Boor for bsplines). Establishes:
- is the flap lune (arc vs its bspline) a needle or a real wedge?
- where do the lines run?
"""
import re
import math

STEP = "/home/z/my-project/3Draper/test/Zentralstaender.stp"

def parse():
    data = open(STEP, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents

def cart(ents, ref):
    """Parse CARTESIAN_POINT #ref -> (x,y,z)."""
    t, a = ents[ref]
    nums = []
    for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", a):
        try:
            nums.append(float(x))
        except ValueError:
            pass
    return tuple(nums[-3:]) if len(nums) >= 3 else (None, None, None)

def refs_of(ent):
    return [int(x) for x in re.findall(r"#(\d+)", ent[1])]

def main():
    ents = parse()
    for ec_id in [5564, 5565, 5577, 5578, 5579, 5580, 5581, 5582,
                  5583, 5584, 5585, 5586, 5587, 5588,
                  5589, 5590, 5591, 5592, 5593, 5594]:
        ec = ents.get(ec_id)
        if not ec:
            continue
        rr = refs_of(ec)
        geom = ents.get(rr[-1]) if rr else None
        if not geom:
            continue
        gt = geom[0]
        if gt == "CIRCLE":
            # CIRCLE('',#axis,r) ; axis=#axis_placement(#cart,#dir1,#dir2)
            nums = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", geom[1])]
            r = nums[-1]
            ax = ents[refs_of(geom)[0]]
            c = cart(ents, refs_of(ax)[0])
            print(f"edge#{ec_id} CIRCLE r={r:.4f} center={c}")
        elif gt == "LINE":
            ax = ents[refs_of(geom)[0]]
            c = cart(ents, refs_of(ax)[0])
            d = ents[refs_of(ax)[1]]
            dv = tuple(float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", d[1])[-3:])
            print(f"edge#{ec_id} LINE through {c} dir {dv}")
        elif gt.startswith("B_SPLINE_CURVE_WITH_KNOTS"):
            # the curve entity's own refs = control points (+ maybe none else)
            crefs = refs_of(geom)
            cps = [cart(ents, r) for r in crefs]
            nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", geom[1])
            print(f"edge#{ec_id} {gt} cps={len(cps)} rawnums_head={nums[:6]}:")
            for i, p in enumerate(cps):
                if p[0] is None:
                    print(f"    cp[{i}] = <{ents[crefs[i]][0]}>")
                else:
                    print(f"    cp[{i}] = ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
        else:
            print(f"edge#{ec_id} {gt}")

    # also: vertices of edge #5564 and #5577 (EDGE_CURVE refs: start vert, end vert, curve)
    print("\n=== EDGE_CURVE vertex roles (edge#, vstart, vend, curve) ===")
    for ec_id in [5564, 5577, 5585, 5589, 5590]:
        ec = ents.get(ec_id)
        rr = refs_of(ec)
        # EDGE_CURVE(name, v1, v2, curve, same_sense)
        v1, v2, cv = rr[0], rr[1], rr[2]
        p1 = vert_pos(ents, v1)
        p2 = vert_pos(ents, v2)
        print(f"edge#{ec_id}: v#{v1} {p1} -> v#{v2} {p2} (curve #{cv})")

def vert_pos(ents, vid):
    t, a = ents[vid]
    rr = [int(x) for x in re.findall(r"#(\d+)", a)]
    pt = cart(ents, rr[0])
    return f"({pt[0]:.4f},{pt[1]:.4f},{pt[2]:.4f})"

if __name__ == "__main__":
    main()
