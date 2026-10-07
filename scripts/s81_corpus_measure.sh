#!/usr/bin/env bash
# s81 A/B: flip_zero_area_ears WINDING FIX (default ON) vs pristine s80
# (recorded numbers). Captures per-BREP pair summaries + watertight lines
# for every corpus file.
set -u
cd /home/z/my-project/3Draper
OUT=/home/z/my-project/scripts/s81_corpus
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
  ./target/release/fold_face_probe "$f" \
    > "$OUT/${name}.out" 2> "$OUT/${name}.err" || true
  grep -oE "BREP#[0-9]+: [0-9]+ pairs[^,]*, [0-9]+ sub-tol weld-noise, [0-9]+ real\) eff_tol=[0-9.]+" \
    "$OUT/${name}.out" | sed 's/eff_tol.*//' > "$OUT/${name}.pairs" || true
  cat "$OUT/${name}.pairs"
  awk -F'[ ,]' '{for(i=1;i<=NF;i++) if($i=="real)") {gsub(/\)/,"",$(i-1)); s+=$(i-1)}} END {print "  TOTAL real:", s}' "$OUT/${name}.pairs"
  grep -c "not watertight" "$OUT/${name}.err" > "$OUT/${name}.wtcount" 2>/dev/null || true
  echo "  not-watertight lines: $(cat "$OUT/${name}.wtcount")"
done
