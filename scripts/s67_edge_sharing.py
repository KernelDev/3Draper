#!/usr/bin/env python3
"""s67 diag 10: GLOBAL edge-sharing census of HOUSING BREP#47598.

Collects ADVANCED_FACE -> EDGE_CURVE for ALL faces of the BREP
(via ROOT/HOLE structure — faces found by scanning the assembly
tree is hard; instead scan ALL faces whose ADVANCED_FACE id is in
the facemap = the 265 converted faces). Then:
 - how many edge_curves are used by 1 / 2 / 3+ faces
 - for the debt faces: are their edges shared, and with whom
"""
import re
from collections import defaultdict

STP = "/home/z/my-project/3Draper/test/drill_top.stp"
FACEMAP = ("/home/z/my-project/scripts/s67_objs/"
           "brep3_HOUSING (BREP#47598).facemap")
DEBT = [125, 214, 130, 261, 212, 127, 158, 160, 162, 164, 166,
        196, 198, 200, 202, 204, 236, 238, 12, 14, 69]


def main():
    data = open(STP, errors="replace").read()
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;",
                         data, re.S):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    def refs(a):
        return [int(x) for x in re.findall(r"#(\d+)", a)]

    fid2step = {}
    stype = {}
    for line in open(FACEMAP):
        if line.startswith("f "):
            p = line.split()
            fid2step[int(p[1])] = int(p[-2])
            stype[int(p[1])] = " ".join(p[2:-2])

    ec_owner = defaultdict(set)
    fid_edges = {}
    for fid, sid in fid2step.items():
        if sid not in ents:
            continue
        edges = []
        for b in refs(ents[sid][1]):
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
                            edges.append(ec)
                            ec_owner[ec].add(fid)
        fid_edges[fid] = set(edges)

    hist = defaultdict(int)
    for ec, owners in ec_owner.items():
        hist[len(owners)] += 1
    print(f"HOUSING edge_curves: total {len(ec_owner)}, "
          f"owners-hist {dict(sorted(hist.items()))}")

    # debt faces: edge sharing profile
    print("\ndebt faces: #edges / #shared(2+) / partners")
    for fid in DEBT:
        es = fid_edges.get(fid, set())
        sh = [e for e in es if len(ec_owner[e]) >= 2]
        partners = defaultdict(int)
        for e in sh:
            for o in ec_owner[e]:
                if o != fid:
                    partners[o] += 1
        top = sorted(partners.items(), key=lambda kv: -kv[1])[:3]
        print(f"  f{fid:4d} [{stype.get(fid,'?')[:12]:12s}] "
              f"{len(es):3d} edges, {len(sh):3d} shared | "
              + ", ".join(f"f{o}:{n}" for o, n in top))


if __name__ == "__main__":
    main()
