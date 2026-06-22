#!/usr/bin/env bash
# Deploy the draper-viewer WASM demo to the gh-pages branch.
# This is the same build the GitHub Actions workflow produces, but run
# locally so the user sees the change immediately without waiting for CI.
#
# The gh-pages branch is a minimal orphan branch containing only 4 files:
#   .nojekyll, draper-viewer.js, draper-viewer_bg.wasm, index.html
# We clone it fresh into /tmp/gh-deploy, swap in the newly-built wasm/js,
# commit, and push. The existing index.html is preserved (it's only
# updated when the user explicitly changes the demo HTML).
#
# Usage:
#   scripts/deploy_gh_pages.sh                       # default commit msg
#   scripts/deploy_gh_pages.sh "custom commit msg"  # custom commit msg
set -euo pipefail

PROJECT_ROOT="/home/z/my-project"
export PATH="/home/z/.cargo/bin:/home/z/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin:$PATH"

cd "$PROJECT_ROOT"

COMMIT_MSG="${1:-deploy: WASM rebuild from $(git rev-parse --short HEAD)}"

echo "==> Building WASM release (web-deploy feature)..."
cargo build -p draper-viewer \
    --release \
    --no-default-features \
    --features web-deploy \
    --target wasm32-unknown-unknown

echo "==> Running wasm-bindgen --target web..."
rm -rf /tmp/wasm-bindgen-out
mkdir -p /tmp/wasm-bindgen-out
wasm-bindgen \
    --target web \
    --no-typescript \
    --out-dir /tmp/wasm-bindgen-out \
    "$PROJECT_ROOT/target/wasm32-unknown-unknown/release/draper-viewer.wasm"

WASM_FILE="/tmp/wasm-bindgen-out/draper-viewer_bg.wasm"
JS_FILE="/tmp/wasm-bindgen-out/draper-viewer.js"
echo "    wasm: $(stat -c '%s' "$WASM_FILE") bytes"
echo "    js:   $(stat -c '%s' "$JS_FILE") bytes"

# Get the authenticated remote URL from the main repo
REMOTE_URL=$(git remote get-url origin)

# Clone gh-pages fresh into /tmp
echo "==> Cloning gh-pages branch into /tmp/gh-deploy..."
rm -rf /tmp/gh-deploy
git clone --branch gh-pages --single-branch "$REMOTE_URL" /tmp/gh-deploy 2>&1 | tail -3
cd /tmp/gh-deploy
git remote set-url origin "$REMOTE_URL"

# Swap in new wasm + js (keep existing index.html)
cp "$WASM_FILE" /tmp/gh-deploy/draper-viewer_bg.wasm
cp "$JS_FILE"   /tmp/gh-deploy/draper-viewer.js

git add -A
git -c user.email=dev@3draper.local -c user.name="3Draper Dev" \
    commit -m "$COMMIT_MSG" 2>&1 | tail -3
git push origin gh-pages 2>&1 | tail -3

echo "==> Deployed to https://kerneldev.github.io/3Draper/"
