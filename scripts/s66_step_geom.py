#!/usr/bin/env python3
"""s66 diag 4b: raw STEP geometry of BREP#1083 — resolve nested refs.
ADVANCED_FACE #169x -> surface #51x; EDGE_CURVE #55xx -> curve #74x.
"""
import re
import math

STEP = "/home/z/my-project/3Draper/test/Zentralstaender.stp"


def get_ents():
    data = open(STEP, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents


def nums(s):
    out = []
    for x in re.findall(r"[-\d.eE+]+", s):
        try:
            out.append(float(x))
        except ValueError:
            pass
    return out


def refs(s):
    return [int(x) for x in re.findall(r"#(\d+)", s)]


def placement(ents, pid):
    """AXIS2_PLACEMENT_3D -> (origin, z_axis, x_axis) or None."""
    if ents.get(pid, ("", ""))[0] != "AXIS2_PLACEMENT_3D":
        return None
    rr = refs(ents[pid][1])
    if len(rr) < 2:
        return None
    org = nums(ents[rr[0]][1]) if ents[rr[0]][0] == "CARTESIAN_POINT" else None
    z = nums(ents[rr[1]][1]) if ents[rr[1]][0] == "DIRECTION" else None
    x = nums(ents[rr[2]][1]) if len(rr) > 2 and ents[rr[2]][0] == "DIRECTION" else None
    if org is None:
        return None
    return org, z or [0, 0, 1], x or [1, 0, 0]


def main():
    ents = get_ents()

    def surf_of_face(fid):
        rr = refs(ents[fid][1])
        sid = rr[-2] if len(rr) >= 2 else rr[-1]
        return sid

    faces = {1692: "f1", 1693: "f2", 1694: "f3", 1695: "f4", 1696: "f5",
             1697: "f6", 1698: "f7", 1699: "f8", 1700: "f9", 1701: "f10",
             1702: "f11", 1703: "f12"}
    print("=== surfaces ===")
    sids = {}
    for fid, name in faces.items():
        sid = surf_of_face(fid)
        sids[name] = sid
        s = ents[sid]
        rr = refs(s[1])
        pl = placement(ents, rr[0])
        vals = nums(s[1])
        print(f"\n{name}: #{sid} {s[0]}")
        if pl:
            org, z, x = pl
            print(f"  origin=({org[0]:.3f},{org[1]:.3f},{org[2]:.3f}) "
                  f"z=({z[0]:.3f},{z[1]:.3f},{z[2]:.3f}) "
                  f"x=({x[0]:.3f},{x[1]:.3f},{x[2]:.3f})")
            print(f"  radii params: {vals[-3:]}")

    circles = {
        5523: "f1|f3", 5524: "f1|f11", 5525: "f2|f4", 5526: "f2|f11",
        5527: "f3|f5", 5528: "f4|f6", 5529: "f5|f7", 5530: "f6|f8",
        5531: "f7|f10", 5532: "f8|f9", 5533: "f9|f12", 5534: "f10|f12",
    }
    print("\n=== circles (EDGE_CURVE -> CIRCLE) ===")
    for cid, users in circles.items():
        rr = refs(ents[cid][1])
        geom = rr[-2] if ents[rr[-1]][0] not in ("CIRCLE", "ELLIPSE") else rr[-1]
        # EDGE_CURVE(v,v,curve,flag): curve is 3rd ref
        curve_id = [x for x in rr if ents[x][0] in ("CIRCLE", "ELLIPSE")]
        curve_id = curve_id[0] if curve_id else geom
        c = ents[curve_id]
        crr = refs(c[1])
        pl = placement(ents, crr[0])
        vals = nums(c[1])
        # vertex pts
        vpts = []
        for x in rr:
            if ents[x][0] == "VERTEX_POINT":
                pref = refs(ents[x][1])[0]
                vpts.append(tuple(nums(ents[pref][1])))
        vline = ""
        if vpts:
            same = math.dist(vpts[0], vpts[-1]) < 1e-9
            vline = (f" v0={tuple(round(c,3) for c in vpts[0])}"
                     + (" FULL-CIRCLE (v0==v1)" if same else " OPEN"))
        if pl:
            org, z, x = pl
            print(f"#{cid} ({users}) -> CIRCLE#{curve_id} r={vals[-1]:.4f} "
                  f"center=({org[0]:.3f},{org[1]:.3f},{org[2]:.3f}) "
                  f"axis=({z[0]:.3f},{z[1]:.3f},{z[2]:.3f}){vline}")
        else:
            print(f"#{cid} ({users}) -> CIRCLE#{curve_id} raw={c[1][:100]}")


if __name__ == "__main__":
    main()
