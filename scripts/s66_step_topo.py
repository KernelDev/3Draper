#!/usr/bin/env python3
"""s66 diag 2: STEP topology of BREP#1083 ELBETAETIGUNG — the f1/f2
(region A) and f5/f6 (region B) unwelded 31-edge arc loops + stray
long edges on f4/f7/f8/f9/f10.

Adapted from s65_step_topo.py: surface types per face, every
EDGE_CURVE (type + sharers + radius), digon detection, non-manifold
check.
"""
import re
from collections import defaultdict

STEP = "/home/z/my-project/3Draper/test/Zentralstaender.stp"
BREP = 1083


def main():
    data = open(STEP, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    shell_id = int(re.search(r"#(\d+)", ents[BREP][1]).group(1))
    face_ids = [int(x) for x in re.findall(r"#(\d+)", ents[shell_id][1])]
    print(f"BREP#{BREP}: shell #{shell_id}, {len(face_ids)} faces")

    def curve_of(ec_id):
        ec = ents.get(ec_id)
        if not ec:
            return "?", -1, ""
        refs = [int(x) for x in re.findall(r"#(\d+)", ec[1])]
        geom = refs[-1] if refs else -1
        cv = ents.get(geom, ("?", ""))
        rad = ""
        if cv[0] == "CIRCLE":
            nums = re.findall(r"[-\d.eE+]+", cv[1])
            try:
                rad = f" r={float(nums[-1]):.4f}"
            except Exception:
                pass
        if cv[0] == "ELLIPSE":
            nums = re.findall(r"[-\d.eE+]+", cv[1])
            try:
                rad = f" r=({float(nums[-2]):.4f},{float(nums[-1]):.4f})"
            except Exception:
                pass
        return cv[0], geom, rad

    edge_faces = defaultdict(list)
    face_edges = defaultdict(list)
    surf_of = {}
    for i, fid in enumerate(face_ids):
        flocal = i + 1
        e = ents[fid]
        refs = [int(x) for x in re.findall(r"#(\d+)", e[1])]
        surf_of[flocal] = ents.get(refs[-1], ("?", ""))[0] if refs else "?"
        for bid in refs[:-1]:
            be = ents.get(bid)
            if not be:
                continue
            lid = int(re.findall(r"#(\d+)", be[1])[0])
            loop = ents.get(lid)
            if not loop:
                continue
            if loop[0] == "VERTEX_LOOP":
                face_edges[flocal].append((None, "VERTEX_LOOP", ""))
                continue
            for oe in re.findall(r"#(\d+)", loop[1]):
                oee = ents.get(int(oe))
                if oee and oee[0] == "ORIENTED_EDGE":
                    ec_id = int(re.findall(r"#(\d+)", oee[1])[-1])
                    ctype, geom, rad = curve_of(ec_id)
                    edge_faces[ec_id].append(flocal)
                    face_edges[flocal].append((ec_id, ctype, rad))

    print("\nlocal  step_face  surface")
    for i in sorted(surf_of):
        print(f"  f{i}: #{face_ids[i-1]} {surf_of[i]}")

    print(f"\nedge census: {len(edge_faces)} unique EDGE_CURVEs")
    shar = {e: sorted(set(fs)) for e, fs in edge_faces.items()}
    multi3 = {e: fs for e, fs in shar.items() if len(fs) > 2}
    print(f"shared by >2 faces (NON-MANIFOLD STEP): {len(multi3)}")
    for e, fs in sorted(multi3.items()):
        ct, gm, rad = curve_of(e)
        print(f"  edge#{e} {ct}{rad} shared_by={fs} (uses: {edge_faces[e]})")

    # digons (2-edge faces) — the s65 lune class
    print("\ndigon faces (<=2 edges):")
    for f, eds in sorted(face_edges.items()):
        uniq = [e for e in set(x[0] for x in eds if x[0] is not None)]
        if 0 < len(uniq) <= 2:
            print(f"  f{f} ({surf_of.get(f)}): {len(eds)} edge uses, "
                  f"unique {len(uniq)}")
            for ec_id, ctype, rad in eds:
                sharers = shar.get(ec_id, [])
                print(f"    edge#{ec_id} {ctype}{rad} shared_by={sharers}")

    # full boundary listing of affected faces
    targets = (1, 2, 4, 5, 6, 7, 8, 9, 10)
    for face in targets:
        print(f"\n== f{face} (#{face_ids[face-1]} {surf_of.get(face)}) edges:")
        seen = set()
        for ec_id, ctype, rad in face_edges.get(face, []):
            if ec_id is None:
                print(f"  {ctype}")
                continue
            if ec_id in seen:
                continue
            seen.add(ec_id)
            sharers = shar.get(ec_id, [])
            multi_note = (f" x{edge_faces[ec_id].count(face)}"
                          if edge_faces[ec_id].count(face) > 1 else "")
            print(f"  edge#{ec_id} {ctype}{rad:16s} "
                  f"shared_by={sharers}{multi_note}")


if __name__ == "__main__":
    main()
