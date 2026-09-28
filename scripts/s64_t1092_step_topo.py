#!/usr/bin/env python3
"""s64 diag: STEP topology of BREP#1092 — face/edge sharing for f29/f32/f3/f12.

Parses Zentralstaender.stp, walks the CLOSED_SHELL of #1092 (33 faces),
resolves ADVANCED_FACE -> FACE_BOUND -> EDGE_LOOP -> ORIENTED_EDGE ->
EDGE_CURVE -> curve type, and prints the edge-sharing table.
"""
import re
from collections import defaultdict

STEP = "/home/z/my-project/3Draper/test/Zentralstaender.stp"

def main():
    data = open(STEP, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    shell_id = int(re.search(r"#(\d+)", ents[1092][1]).group(1))
    face_ids = [int(x) for x in re.findall(r"#(\d+)", ents[shell_id][1])]
    print(f"BREP#1092: shell #{shell_id}, {len(face_ids)} faces")

    def curve_of(ec_id):
        ec = ents.get(ec_id)
        if not ec:
            return "?", -1
        refs = [int(x) for x in re.findall(r"#(\d+)", ec[1])]
        geom = refs[-1] if refs else -1
        cv = ents.get(geom, ("?", ""))
        # for trimmed curves the geometry may be a ref to the basis; one level
        return cv[0], geom

    edge_faces = defaultdict(set)
    face_edges = defaultdict(list)
    surf_of = {}
    for i, fid in enumerate(face_ids):
        flocal = i + 1
        e = ents[fid]
        refs = [int(x) for x in re.findall(r"#(\d+)", e[1])]
        surf_of[flocal] = ents.get(refs[-1], ("?", ""))[0] if refs else "?"
        for bid in refs[:-1]:  # bounds
            be = ents.get(bid)
            if not be:
                continue
            # FACE_OUTER_BOUND('',#loop,.T.)
            lid = int(re.findall(r"#(\d+)", be[1])[0])
            loop = ents.get(lid)
            if not loop:
                continue
            if loop[0] == "VERTEX_LOOP":
                face_edges[flocal].append((None, "VERTEX_LOOP"))
                continue
            for oe in re.findall(r"#(\d+)", loop[1]):
                oee = ents.get(int(oe))
                if oee and oee[0] == "ORIENTED_EDGE":
                    ec_id = int(re.findall(r"#(\d+)", oee[1])[-1])
                    ctype, geom = curve_of(ec_id)
                    edge_faces[ec_id].add(flocal)
                    face_edges[flocal].append((ec_id, ctype))

    print("\nlocal  step_face  surface")
    for i in sorted(surf_of):
        if i in (1, 3, 12, 20, 21, 29, 32):
            print(f"  f{i}: #{face_ids[i-1]} {surf_of[i]}")

    print(f"\nedge census: {len(edge_faces)} unique EDGE_CURVEs")
    multi = {e: fs for e, fs in edge_faces.items() if len(fs) > 1}
    print(f"shared by >1 face: {len(multi)}")

    for face in (29, 32, 3, 12):
        print(f"\n== f{face} ({surf_of.get(face)}) edges:")
        seen = set()
        for ec_id, ctype in face_edges.get(face, []):
            if ec_id in seen:
                continue
            seen.add(ec_id)
            sharers = sorted(edge_faces.get(ec_id, set()))
            print(f"  edge#{ec_id} {ctype:28s} shared_by={sharers}")

    # full surface map for reference
    print("\nall faces:")
    for i in sorted(surf_of):
        print(f"  f{i}: {surf_of[i]}")

if __name__ == "__main__":
    main()
