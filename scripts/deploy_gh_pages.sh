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
# inject the git revision into index.html, add cache busters, commit, and push.
#
# Usage:
#   scripts/deploy_gh_pages.sh                       # default commit msg
#   scripts/deploy_gh_pages.sh "custom commit msg"  # custom commit msg
set -euo pipefail

PROJECT_ROOT="/home/z/my-project"
export PATH="/home/z/.cargo/bin:$PATH"

cd "$PROJECT_ROOT"

GIT_HASH=$(git rev-parse --short HEAD)
COMMIT_MSG="${1:-deploy: WASM rebuild from $GIT_HASH}"

echo "==> Git revision: $GIT_HASH"

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
rm -rf /tmp/wasm-bindgen-out-worker
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
cp "$VIEWER_WASM" /tmp/gh-deploy/draper-viewer_bg.wasm
cp "$VIEWER_JS"   /tmp/gh-deploy/draper-viewer.js
cp "$WORKER_WASM" /tmp/gh-deploy/draper-worker_bg.wasm
cp "$WORKER_JS"   /tmp/gh-deploy/draper-worker.js

# Copy worker JS files
cp "$PROJECT_ROOT/crates/draper-viewer/worker.js" /tmp/gh-deploy/worker.js
cp "$PROJECT_ROOT/crates/draper-viewer/worker-bridge.js" /tmp/gh-deploy/worker-bridge.js

# ── Inject git revision into index.html ──────────────────────────────
echo "==> Injecting revision $GIT_HASH into index.html..."
INDEX_HTML=/tmp/gh-deploy/index.html

# Remove any old revision badges from previous deploys
sed -i '/build: [0-9a-f]\{7\}[\+]*"/d' "$INDEX_HTML"

# Update page title to include revision
sed -i "s|<title>3Draper[^<]*</title>|<title>3Draper [$GIT_HASH] — 3D Geometric Kernel</title>|" "$INDEX_HTML"

# Add revision badge in loading screen (after "Loading WebAssembly module..." status)
sed -i "/<div class=\"status\">Loading WebAssembly module\.\.\.<\/div>/a\\        <div class=\"parallel-badge\" style=\"margin-top:4px;font-size:11px;color:#8ca0b4;\">build: $GIT_HASH</div>" "$INDEX_HTML"

# Update cache buster query parameters to WASM/JS URLs (replace any existing ?v=)
sed -i "s|draper-viewer\.js?v=[0-9a-f]*|draper-viewer.js?v=$GIT_HASH|g" "$INDEX_HTML"
sed -i "s|draper-viewer_bg\.wasm?v=[0-9a-f]*|draper-viewer_bg.wasm?v=$GIT_HASH|g" "$INDEX_HTML"
# Also add cache busters if they don't exist yet
sed -i "s|'./draper-viewer.js'|'./draper-viewer.js?v=$GIT_HASH'|g" "$INDEX_HTML"
sed -i "s|'./draper-viewer_bg.wasm'|'./draper-viewer_bg.wasm?v=$GIT_HASH'|g" "$INDEX_HTML"

echo "    revision in title: $(grep -c "$GIT_HASH" "$INDEX_HTML") occurrences"

git add -A
git -c user.email=dev@3draper.local -c user.name="3Draper Dev" \
    commit -m "$COMMIT_MSG" 2>&1 | tail -3
git push origin gh-pages 2>&1 | tail -3

echo "==> Deployed to https://kerneldev.github.io/3Draper/ [build: $GIT_HASH]"
