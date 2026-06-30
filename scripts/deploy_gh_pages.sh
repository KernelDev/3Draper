#!/usr/bin/env bash
# Deploy the draper-viewer WASM demo to the gh-pages branch.
# This is the same build the GitHub Actions workflow produces, but run
# locally so the user sees the change immediately without waiting for CI.
#
# The gh-pages branch contains:
#   .nojekyll, draper-viewer.js, draper-viewer_bg.wasm,
#   draper-worker.js, draper-worker_bg.wasm, worker.js, worker-bridge.js,
#   index.html
#
# We clone it fresh into /tmp/gh-deploy, swap in the newly-built wasm/js,
# commit, and push.
#
# Usage:
#   scripts/deploy_gh_pages.sh                       # default commit msg
#   scripts/deploy_gh_pages.sh "custom commit msg"  # custom commit msg
set -euo pipefail

PROJECT_ROOT="/home/z/my-project"
export PATH="/home/z/.cargo/bin:/home/z/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin:$PATH"

cd "$PROJECT_ROOT"

COMMIT_MSG="${1:-deploy: WASM rebuild from $(git rev-parse --short HEAD)}"

echo "==> Building viewer WASM release (web-deploy feature)..."
cargo build -p draper-viewer \
    --release \
    --no-default-features \
    --features web-deploy \
    --target wasm32-unknown-unknown

echo "==> Building worker WASM release..."
cargo build -p draper-worker \
    --release \
    --target wasm32-unknown-unknown

echo "==> Running wasm-bindgen for viewer..."
rm -rf /tmp/wasm-bindgen-out
mkdir -p /tmp/wasm-bindgen-out
wasm-bindgen \
    --target web \
    --no-typescript \
    --out-dir /tmp/wasm-bindgen-out \
    "$PROJECT_ROOT/target/wasm32-unknown-unknown/release/draper-viewer.wasm"

echo "==> Running wasm-bindgen for worker..."
mkdir -p /tmp/wasm-bindgen-out-worker
wasm-bindgen \
    --target web \
    --no-typescript \
    --out-dir /tmp/wasm-bindgen-out-worker \
    "$PROJECT_ROOT/target/wasm32-unknown-unknown/release/draper_worker.wasm"

VIEWER_WASM="/tmp/wasm-bindgen-out/draper-viewer_bg.wasm"
VIEWER_JS="/tmp/wasm-bindgen-out/draper-viewer.js"
WORKER_WASM="/tmp/wasm-bindgen-out-worker/draper_worker_bg.wasm"
WORKER_JS="/tmp/wasm-bindgen-out-worker/draper_worker.js"

echo "    viewer wasm: $(stat -c '%s' "$VIEWER_WASM") bytes"
echo "    viewer js:   $(stat -c '%s' "$VIEWER_JS") bytes"
echo "    worker wasm: $(stat -c '%s' "$WORKER_WASM") bytes"
echo "    worker js:   $(stat -c '%s' "$WORKER_JS") bytes"

# Get the authenticated remote URL from the main repo
REMOTE_URL=$(git remote get-url origin)

# Clone gh-pages fresh into /tmp
echo "==> Cloning gh-pages branch into /tmp/gh-deploy..."
rm -rf /tmp/gh-deploy
git clone --branch gh-pages --single-branch "$REMOTE_URL" /tmp/gh-deploy 2>&1 | tail -3
cd /tmp/gh-deploy
git remote set-url origin "$REMOTE_URL"

# Swap in new wasm + js
# Note: wasm-bindgen produces draper_worker_*.wasm/js (underscores),
# but the web worker JS expects draper-worker.* (hyphens).
# We rename during copy to match the expected names.
cp "$VIEWER_WASM" /tmp/gh-deploy/draper-viewer_bg.wasm
cp "$VIEWER_JS"   /tmp/gh-deploy/draper-viewer.js
cp "$WORKER_WASM" /tmp/gh-deploy/draper-worker_bg.wasm
cp "$WORKER_JS"   /tmp/gh-deploy/draper-worker.js

# Copy worker JS files
cp "$PROJECT_ROOT/crates/draper-viewer/worker.js" /tmp/gh-deploy/worker.js
cp "$PROJECT_ROOT/crates/draper-viewer/worker-bridge.js" /tmp/gh-deploy/worker-bridge.js

git add -A
git -c user.email=dev@3draper.local -c user.name="3Draper Dev" \
    commit -m "$COMMIT_MSG" 2>&1 | tail -3
git push origin gh-pages 2>&1 | tail -3

echo "==> Deployed to https://kerneldev.github.io/3Draper/"
