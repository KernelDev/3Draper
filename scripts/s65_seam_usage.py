#!/usr/bin/env python3
"""s65 diag 12: face-usage census for the seam pairs rejected by the
shape guard — are the paired curves 'loners' (1 face each, the glue is
a repair) or 'paired' (2 faces each, the glue creates a 3-way)?
Runs on drill_top for the rejected pairs + on Zentralstaender for the
lune pairs (arc/lip) as the control.
"""
import re
from collections import defaultdict

def parse(path):
    data = open(path, encoding="utf-8", errors="replace").read()
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))
    return ents

def edge_face_usage(ents, shell_id):
    """edge step_id -> set of (face_step_id) using it."""
    face_ids = [int(x) for x in re.findall(r"#(\d+)", ents[shell_id][1])]
    usage = defaultdict(set)
    for fid in face_ids:
        rr = [int(x) for x in re.findall(r"#(\d+)", ents[fid][1])]
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
                    usage[ec_id].add(fid)
    return usage, face_ids

def main_drill():
    ents = parse("/home/z/my-project/3Draper/test/drill_top.stp")
    # BREP #47598 shell
    pairs = [(831, 833), (837, 839), (932, 934)]
    shell_id = int(re.search(r"#(\d+)", ents[47598][1]).group(1))
    usage, faces = edge_face_usage(ents, shell_id)
    print(f"drill BREP #47598: shell #{shell_id}, {len(faces)} faces")
    for a, b in pairs:
        fa = usage.get(a, set())
        fb = usage.get(b, set())
        print(f"  #{a}: faces {sorted(fa)}   #{b}: faces {sorted(fb)}")

    # also the bigger BREP (with #9766 etc.)
    shell_id2 = int(re.search(r"#(\d+)", ents[62542][1]).group(1))
    usage2, faces2 = edge_face_usage(ents, shell_id2)
    pairs2 = [(9766, 9535), (9744, 10630), (11864, 9631)]
    print(f"drill BREP #62542: shell #{shell_id2}, {len(faces2)} faces")
    for a, b in pairs2:
        fa = usage2.get(a, set())
        fb = usage2.get(b, set())
        # also check across BOTH shells
        fa_all = fa | usage.get(a, set())
        fb_all = fb | usage.get(b, set())
        print(f"  #{a}: faces {sorted(fa)} (+other shell {sorted(fa_all - fa)})   "
              f"#{b}: faces {sorted(fb)} (+other shell {sorted(fb_all - fb)})")

def main_z():
    ents = parse("/home/z/my-project/3Draper/test/Zentralstaender.stp")
    shell_id = int(re.search(r"#(\d+)", ents[1086][1]).group(1))
    usage, faces = edge_face_usage(ents, shell_id)
    print(f"\nZ BREP #1086: shell #{shell_id}, {len(faces)} faces")
    for a, b in [(5564, 5577), (5569, 5578), (5565, 5580)]:
        fa = usage.get(a, set())
        fb = usage.get(b, set())
        print(f"  #{a}: faces {sorted(fa)}   #{b}: faces {sorted(fb)}")

if __name__ == "__main__":
    main_drill()
    main_z()
