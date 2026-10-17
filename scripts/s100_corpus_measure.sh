#!/usr/bin/env bash
# s100 A/B: seam_bornfold_split (default ON) vs kill-switch OFF, full corpus.
# Captures per-BREP pair summaries (sorted), REAL totals, BUG-line counts.
set -u
cd /home/z/my-project/3Draper
OUT=/home/z/my-project/scripts/s100_corpus
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
MODES=("OFF" "ON")
for mode in "${MODES[@]}"; do
  if [ "$mode" = "OFF" ]; then export DRAPPER_SEAM_FOLD_SPLIT=0; else unset DRAPPER_SEAM_FOLD_SPLIT; fi
  for f in "${FILES[@]}"; do
    name=$(basename "$f" .stp)
    ./target/release/fold_face_probe "$f" \
      > "$OUT/${name}.${mode}.out" 2> "$OUT/${name}.${mode}.err" || true
    grep -E "^--- " "$OUT/${name}.${mode}.out" | sort > "$OUT/${name}.${mode}.pairs"
    real_total=$(awk -F'[ ,]' '{for(i=1;i<=NF;i++) if($i=="real)") {gsub(/\)/,"",$(i-1)); s+=$(i-1)}} END {print s}' "$OUT/${name}.${mode}.pairs")
    subtol_total=$(awk -F'[ ,]' '{for(i=1;i<=NF;i++) if($i=="weld-noise,") {gsub(/\)/,"",$(i-1)); s+=$(i-1)}} END {print s}' "$OUT/${name}.${mode}.pairs")
    wt=$(grep -c "not watertight" "$OUT/${name}.${mode}.err" 2>/dev/null || echo 0)
    echo "$mode $name real=$real_total subtol=$subtol_total not_wt_lines=$wt"
  done
done
echo "=== DIFFS (OFF vs ON) ==="
for f in "${FILES[@]}"; do
  name=$(basename "$f" .stp)
  if diff -q "$OUT/${name}.OFF.pairs" "$OUT/${name}.ON.pairs" > /dev/null 2>&1; then
    echo "$name: pairs IDENTICAL"
  else
    echo "$name: pairs DIFFER:"
    diff "$OUT/${name}.OFF.pairs" "$OUT/${name}.ON.pairs" | sed 's/^/    /'
  fi
done
