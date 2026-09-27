#!/usr/bin/env python3
"""s63 diag: compare STEP CLOSED_SHELL face list of a BREP vs triangulated faces.

Parses the STEP file (line-based, entity refs #N), finds MANIFOLD_SOLID_BREP
with the given id, walks to its CLOSED_SHELL, lists ADVANCED_FACE ids with
their surface types, and diffs against the per-face OBJ dump filenames.
"""
import glob
import os
import re
import sys

STEP = "/home/z/my-project/3Draper/test/Zentralstaender.stp"

def main(brep_id: int, face_dir: str):
    data = open(STEP, encoding="utf-8", errors="replace").read()
    # Join continuation lines: entities end with ';'
    data = re.sub(r"\s*\n\s*", " ", data)
    ents = {}
    for m in re.finditer(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\(([^;]*)\)\s*;", data):
        ents[int(m.group(1))] = (m.group(2), m.group(3))

    # find the solid
    solid = ents.get(brep_id)
    if solid is None:
        print(f"#{brep_id} not found"); return
    print(f"#{brep_id} = {solid[0]}")
    m = re.search(r"#(\d+)", solid[1])
    shell_id = int(m.group(1))
    shell = ents[shell_id]
    print(f"shell #{shell_id} = {shell[0]}({shell[1][:80]}...)")
    face_ids = [int(x) for x in re.findall(r"#(\d+)", shell[1])]
    print(f"faces in shell: {len(face_ids)}")
    # each face ref may be direct ADVANCED_FACE or via list
    step_faces = []
    for fid in face_ids:
        e = ents.get(fid)
        if e is None:
            step_faces.append((fid, "??", "")); continue
        if e[0] == "ADVANCED_FACE":
            # surface is the 2nd-ish ref: ADVANCED_FACE('',(...),#surf,.T.);
            refs = re.findall(r"#(\d+)", e[1])
            surf_id = int(refs[-2]) if len(refs) >= 2 else -1
            surf = ents.get(surf_id, ("?", ""))
            step_faces.append((fid, surf[0], surf_id))
        else:
            step_faces.append((fid, e[0], ""))
    print(f"\n{'STEP face':>10} {'surface':<28} {'surf id':>8}  triangulated?")
    dumped = {}
    for p in glob.glob(os.path.join(face_dir, f"brep{brep_id}_f*.obj")):
        b = os.path.basename(p)
        m = re.match(r"brep\d+_f(\d+)_s(\d+)_(\w+)\.obj", b)
        if m:
            dumped[int(m.group(2))] = (int(m.group(1)), m.group(3))
    n_missing = 0
    for fid, stype, sid in step_faces:
        t = dumped.get(fid)
        mark = f"YES f{t[0]} {t[1]}" if t else "— MISSING"
        if not t:
            n_missing += 1
        print(f"#{fid:>9} {stype:<28} #{sid:<7}  {mark}")
    print(f"\ntriangulated: {len(dumped)} / step faces: {len(step_faces)}; MISSING: {n_missing}")

if __name__ == "__main__":
    main(int(sys.argv[1]), sys.argv[2])
