#!/bin/bash
# Rust toolchain env for 3Draper sandbox (recreated s104 after backup-restore)
export RUSTUP_HOME="/tmp/my-project/.rustup"
export CARGO_HOME="/tmp/my-project/.cargo"
export PATH="/tmp/my-project/.cargo/bin:$PATH"
# default toolchain: stable == rustc 1.98.1 (48a229cea 2026-09-01)
# NOTE s104: named toolchain 1.98.1-x86_64-unknown-linux-gnu is BROKEN after
# backup-restore (empty std lib dir) — use `stable` which is the same rustc
# build with intact std.
