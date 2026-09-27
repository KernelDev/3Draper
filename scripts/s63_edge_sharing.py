#!/usr/bin/env python3
"""s63 diag: STEP edge sharing map for a CLOSED_SHELL.

For every face of the shell: walk FACE_BOUND(s) -> EDGE_LOOP ->
ORIENTED_EDGE -> EDGE_CURVE, note geometry (LINE/CIRCLE/ELLIPSE +
params). Build edge->faces map. Then for the requested face, list its
edges with sharing info: shared-with-which-face, or LONE (used by one
face only = broken shell / unattached rim).
"""
import math
import re
import sys
from collections import defaultdict

STEP = "/home/z/my-project/3Draper/test/Zentralstaender.stp"

def main(shell_face_ids, focus):
    data = open(STEP, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    def refs(s):
        return [int(x) for x in re.findall(r"#(\d+)", s)]

    edge_faces = defaultdict(list)   # edge_curve_id -> [(face_id, orient, loop_role)]
    face_edges = {}                  # face_id -> [(edge_id, orient)]
    face_surf = {}
    for fid in shell_face_ids:
        e = ents[fid]
        assert e[0] == "ADVANCED_FACE", (fid, e[0])
        rr = refs(e[1])
        bounds = rr[:-1]  # last is surface (or second-to-last .T. handling: refs are bounds then surface)
        # ADVANCED_FACE('',(#b1,#b2),#surf,.T.); bounds = all but last ref
        surf = rr[-1]
        face_surf[fid] = surf
        edges = []
        for b in bounds:
            be = ents[b]
            # FACE_OUTER_BOUND('',#loop,.T.) / FACE_BOUND
            loop_id = refs(be[1])[0]
            loop = ents[loop_id]
            if loop[0] == "EDGE_LOOP":
                oe_ids = refs(loop[1])
            elif loop[0] == "VERTEX_LOOP":
                oe_ids = []
            else:
                oe_ids = [loop_id]
            for oe in oe_ids:
                oee = ents[oe]
                # ORIENTED_EDGE('',*,*,#edge,.T./.F.)
                er = refs(oee[1])
                edge = er[-2] if len(er) >= 2 else er[0]
                # last arg .T./.F. not a ref; orient from text
                orient = ".T." in oee[1].rsplit(",", 1)[-1]
                edges.append((edge, orient))
                role = "OUTER" if be[0] == "FACE_OUTER_BOUND" else "BOUND"
                edge_faces[edge].append((fid, orient, role))
        face_edges[fid] = edges

    print(f"focus face #{focus}, surface #{face_surf.get(focus)}")
    surf_of = {}
    for fid in shell_face_ids:
        s = ents[face_surf[fid]]
        surf_of[fid] = s[0]

    # geometry summary of an edge
    def edge_geo(eid):
        e = ents.get(eid)
        if not e:
            return "?"
        rr = refs(e[1])
        if e[0] == "EDGE_CURVE":
            crv = ents.get(rr[-2], ("?", ""))
            kids = crv[1]
            if crv[0] == "CIRCLE":
                nr = refs(kids)
                ax = ents.get(nr[1], ("", "(0)"))
                av = refs(ax[1]) if ax[0] == "AXIS2_PLACEMENT_3D" else []
                pos = ents.get(av[0], ("", ""))
                pv = refs(pos[1]) if pos[0] == "CARTESIAN_POINT" else [0]
                pt = ents.get(pv[0], ("", ""))
                co = re.findall(r"[-\d.eE+]+", pt[1])
                rad = re.findall(r"[-\d.eE+]+", kids)
                return f"CIRCLE r={float(rad[-1]) if rad else -1:.4f} o=({','.join(co[:3])})"
            if crv[0] == "LINE":
                return "LINE"
            return crv[0]
        return e[0]

    for (eid, orient) in face_edges[focus]:
        sharers = edge_faces[eid]
        geo = edge_geo(eid)
        if len(sharers) == 1:
            info = "LONE!"
        else:
            others = [f"#{f}[{surf_of.get(f,'?')[:4]}]" for (f, o, r) in sharers if f != focus]
            info = "shared:" + ",".join(others)
        print(f"  edge#{eid} orient={orient} {geo:<40} {info}")

    # also: global shell health — count lone edges
    lone = [e for e, fs in edge_faces.items() if len(fs) == 1]
    multi = [(e, fs) for e, fs in edge_faces.items() if len(fs) > 2]
    print(f"\nshell health: edges={len(edge_faces)} lone={len(lone)} >2faces={len(multi)}")
    if lone[:12]:
        print("  lone edges:", [f"#{e}({edge_geo(e)}) used_by=#{edge_faces[e][0][0]}" for e in lone[:12]])

if __name__ == "__main__":
    # shell #1113 faces for brep 1086
    face_ids = [1722,1723,1724,1725,1726,1727,1728,1729,1730,1731,1732,1733,1734,1735,1736,1737,1738,1739,1740,1741,1742]
    focus = int(sys.argv[1]) if len(sys.argv) > 1 else 1737
    main(face_ids, focus)
