#!/usr/bin/env python3
"""s65 diag 6: enumerate the vertex-pair groups of BREP#1086 that the
Phase-1 aliaser merges — which 4 step_ids share corner vertex pairs?
"""
import re
from collections import defaultdict

STEP = "/home/z/my-project/3Draper/test/Zentralstaender.stp"
BREP = 1086

def main():
    data = open(STEP, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    shell_id = int(re.search(r"#(\d+)", ents[BREP][1]).group(1))
    face_ids = [int(x) for x in re.findall(r"#(\d+)", ents[shell_id][1])]
    print(f"BREP#{BREP}: shell #{shell_id}, {len(face_ids)} faces")

    def refs(ent):
        return [int(x) for x in re.findall(r"#(\d+)", ent[1])]

    def curve_name(ec_id):
        ec = ents.get(ec_id)
        if not ec:
            return "?"
        rr = refs(ec)
        geom = ents.get(rr[-1]) if rr else None
        if not geom:
            return "?"
        return geom[0]

    # edge step_id -> vertex pair (from EDGE_CURVE: name, v1, v2, curve, sense)
    vp_groups = defaultdict(set)
    for fid in face_ids:
        e = ents[fid]
        rr = refs(e)
        for bid in rr[:-1]:
            be = ents.get(bid)
            if not be:
                continue
            lid = int(re.findall(r"#(\d+)", be[1])[0])
            loop = ents.get(lid)
            if not loop:
                continue
            if loop[0] == "VERTEX_LOOP":
                continue
            for oe in re.findall(r"#(\d+)", loop[1]):
                oee = ents.get(int(oe))
                if oee and oee[0] == "ORIENTED_EDGE":
                    ec_id = int(re.findall(r"#(\d+)", oee[1])[-1])
                    ec = ents.get(ec_id)
                    if ec:
                        err = refs(ec)
                        if len(err) >= 3:
                            v1, v2 = err[0], err[1]
                            key = (min(v1, v2), max(v1, v2))
                            vp_groups[key].add(ec_id)

    print("\nvertex-pair groups with >=2 distinct edge curves:")
    for vp, sids in sorted(vp_groups.items()):
        if len(sids) < 2:
            continue
        types = defaultdict(list)
        for s in sorted(sids):
            types[curve_name(s)].append(s)
        if len(types) > 1:
            print(f"  vp {vp}: {len(sids)} edges")
            for t, ss in types.items():
                print(f"    {t}: {ss}")

if __name__ == "__main__":
    main()
