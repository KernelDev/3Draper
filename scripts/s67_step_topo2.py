#!/usr/bin/env python3
"""s67 diag 9: full STEP topology of debt faces (2-level: FACE_BOUND
-> EDGE_LOOP -> ORIENTED_EDGE -> EDGE_CURVE -> curve + verts).
Shows: loop count, per-loop curve types, shared edges with the
neighbor debt faces (via edge-curve id sets).
"""
import re
from collections import defaultdict

STP = "/home/z/my-project/3Draper/test/drill_top.stp"
FACEMAP = ("/home/z/my-project/scripts/s67_objs/"
           "brep3_HOUSING (BREP#47598).facemap")
TARGETS = [125, 214, 130, 261, 212, 127, 158, 204, 236, 238, 12, 14, 69]


def main():
    data = open(STP, errors="replace").read()
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;",
                         data, re.S):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    def refs(args):
        return [int(x) for x in re.findall(r"#(\d+)", args)]

    fid2step = {}
    for line in open(FACEMAP):
        if line.startswith("f "):
            p = line.split()
            fid2step[int(p[1])] = int(p[-2])

    # edge_curve id -> (curve_type, owning faces)
    ec_owner = defaultdict(set)
    face_edges = {}
    for fid in TARGETS:
        sid = fid2step.get(fid)
        if not sid or sid not in ents:
            continue
        face_edges[fid] = []
        args = ents[sid][1]
        # bounds are inside the FIRST parenthesized list of refs
        for b in refs(args):
            if b not in ents:
                continue
            bt, ba = ents[b]
            if bt not in ("FACE_OUTER_BOUND", "FACE_BOUND"):
                continue
            for lp in refs(ba):
                if lp not in ents or ents[lp][0] != "EDGE_LOOP":
                    continue
                for oe in refs(ents[lp][1]):
                    if oe not in ents or ents[oe][0] != "ORIENTED_EDGE":
                        continue
                    for ec in refs(ents[oe][1]):
                        if ec in ents and ents[ec][0] == "EDGE_CURVE":
                            face_edges[fid].append(ec)
                            ec_owner[ec].add(fid)

    for fid in TARGETS:
        if fid not in face_edges:
            continue
        ecs = face_edges[fid]
        # unique in order
        seen, uniq = set(), []
        for e in ecs:
            if e not in seen:
                seen.add(e); uniq.append(e)
        print(f"\n=== f{fid} (STEP#{fid2step[fid]}): "
              f"{len(uniq)} unique edge_curves ===")
        for e in uniq:
            et, ea = ents[e]
            er = refs(ea)
            ct = "?"
            for r in er:
                if r in ents and ents[r][0] in (
                        "LINE", "CIRCLE", "ELLIPSE",
                        "B_SPLINE_CURVE_WITH_KNOTS", "POLYLINE",
                        "SEAM_CURVE", "SURFACE_CURVE", "INTERSECTION_CURVE"):
                    ct = ents[r][0]
                    cr = r
                    break
            shared = ec_owner[e] - {fid}
            # curve param details
            det = ""
            for r in er:
                if r in ents and ents[r][0] in ("CIRCLE", "LINE", "ELLIPSE"):
                    det = ents[r][1][:80]
                    break
                if r in ents and "B_SPLINE_CURVE" in ents[r][0]:
                    det = "spline " + ents[r][1][:60]
                    break
            print(f"  ec#{e} {ct:28s} shared_with={sorted(shared)} {det}")


if __name__ == "__main__":
    main()
