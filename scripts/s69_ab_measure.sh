#!/usr/bin/env bash
# s69 A/B measurement: TORUS_FILLET_BAND (default ON) vs =0 (s68 baseline).
# Compares probe pair counts + watertight ERROR lines + rescue counts
# per corpus file.
set -u
cd /home/z/my-project/3Draper
OUT=/home/z/my-project/scripts/s69_ab
mkdir -p "$OUT"

FILES=(
  test/Zentralstaender.stp
  test/drill_top.stp
  test/compressor-13920_top.stp
  test/as1-oc-214.stp
  test/transmission_top.stp
  test/brick_thin_round.stp
  test/brick_thin.stp
  test/brick_thin_hole.stp
)

for f in "${FILES[@]}"; do
  name=$(basename "$f" .stp)
  echo "=== $name ==="
  # baseline: kill-switch
  DRAPPER_TORUS_FILLET_BAND=0 ./target/release/fold_face_probe "$f" \
    > "$OUT/${name}.off.out" 2> "$OUT/${name}.off.err" || true
  # new: default ON
  ./target/release/fold_face_probe "$f" \
    > "$OUT/${name}.on.out" 2> "$OUT/${name}.on.err" || true
  # pair lines (stdout)
  rg -o "BREP#\d+: \d+ pairs.*" "$OUT/${name}.off.out" > "$OUT/${name}.off.pairs" 2>/dev/null || true
  rg -o "BREP#\d+: \d+ pairs.*" "$OUT/${name}.on.out" > "$OUT/${name}.on.pairs" 2>/dev/null || true
  echo "-- pairs diff (off vs on):"
  diff "$OUT/${name}.off.pairs" "$OUT/${name}.on.pairs" && echo "   IDENTICAL" || true
  # watertight error lines (stderr)
  rg "not watertight" "$OUT/${name}.off.err" | sort > "$OUT/${name}.off.wt" 2>/dev/null || true
  rg "not watertight" "$OUT/${name}.on.err" | sort > "$OUT/${name}.on.wt" 2>/dev/null || true
  echo "-- watertight lines diff (off vs on):"
  diff "$OUT/${name}.off.wt" "$OUT/${name}.on.wt" && echo "   IDENTICAL" || true
  # rescue acceptances (stderr, new only)
  echo "-- TORUS_FILLET_BAND rescues accepted:"
  rg -c "TORUS_FILLET_BAND rescue" "$OUT/${name}.on.err" || echo "   0"
done
echo "ALL DONE"
