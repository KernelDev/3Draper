#!/bin/bash
# s110 detached build — diagnostic binaries after backup-restore
cd /home/z/my-project/3Draper
source scripts/rust_env.sh
{
  echo "BUILD START $(date)"
  cargo build --release -p draper-diag \
    --bin fold_face_probe \
    --bin angle_check \
    --bin all_files_test \
    --bin sphere_diag \
    --bin sphere_uv_dump \
    --bin s109_eff_tol_probe \
    --bin winding_pair_probe
  echo "EXIT_CODE=$?"
  echo "BUILD END $(date)"
} > /tmp/build_diag.log 2>&1
